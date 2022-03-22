#ifndef RT_EXTRACT_HPP
#define RT_EXTRACT_HPP

#include <embree3/rtcore.h>

#include <lsDomain.hpp>
#include <lsExpand.hpp>
#include <lsMesh.hpp>
#include <lsSmartPointer.hpp>
#include <lsToDiskMesh.hpp>
#include <lsToSurfaceMesh.hpp>

#include <rayBoundCondition.hpp>
#include <rayBoundary.hpp>
#include <rayGeometry.hpp>
#include <rayUtil.hpp>

#include "rtBoundary.hpp"

template <class NumericType, int D> class rtExtract {
  using LSPtr = lsSmartPointer<lsDomain<NumericType, D>>;
  RTCDevice mDevice;
  rayGeometry<NumericType, D> mGeometry;
  NumericType mDiskRadius = 0;
  NumericType mGridDelta = 0;

  LSPtr baseDom = nullptr;
  LSPtr secondDom = nullptr;

  lsSmartPointer<lsMesh<NumericType>> baseMesh = nullptr;
  lsSmartPointer<lsMesh<NumericType>> secondMesh = nullptr;

public:
  rtExtract() : mDevice(rtcNewDevice("hugepages=1")) {}
  rtExtract(LSPtr passedBaseDomain, LSPtr passedSecondDomain)
      : mDevice(rtcNewDevice("hugepages=1")), baseDom(passedBaseDomain),
        secondDom(passedSecondDomain) {}

  ~rtExtract() {
    mGeometry.releaseGeometry();
    rtcReleaseDevice(mDevice);
  }

  lsSmartPointer<lsMesh<>> getResultingMesh() const { return baseMesh; }

  void apply() {
    if (baseDom == nullptr || secondDom == nullptr) {
      lsMessage::getInstance().addError("At least one of the levelsets passed "
                                        "to rtExtract is empty. Aborting.");
      return;
    }

    // Convert levelsets to disk meshes
    baseMesh = lsSmartPointer<lsMesh<NumericType>>::New();
    lsExpand<NumericType, D>(baseDom, 3).apply();
    lsCalculateNormalVectors<NumericType, D>(baseDom).apply();
    lsToSurfaceMesh<NumericType, D>(baseDom, baseMesh).apply();

    // The second mesh is converted to a disk mesh so that it can be used for
    secondMesh = lsSmartPointer<lsMesh<NumericType>>::New();
    lsToDiskMesh<NumericType, D, NumericType, true>(secondDom, secondMesh)
        .apply();

    // Use the second mesh to build the target geometry
    auto nodes = secondMesh->getNodes();
    auto normals = *secondMesh->getCellData().getVectorData("Normals");

    const auto &grid = secondDom->getGrid();
    mGridDelta = grid.getGridDelta();
    mDiskRadius = mGridDelta * rayInternal::DiskFactor;

    mGeometry.initGeometry(mDevice, nodes, normals, mDiskRadius);

    // Calculate the size of the bounding box that surrounds both meshes (+ one
    // grid delta padding)
    const auto boundingBox = calcBoundingBox();

    printf("min: (%f, %f, %f)\n", boundingBox[0][0], boundingBox[0][1],
           boundingBox[0][2]);
    printf("max: (%f, %f, %f)\n", boundingBox[1][0], boundingBox[1][1],
           boundingBox[1][2]);

    initMemoryFlags();

    auto mBoundary = rtBoundary<NumericType, D>(mDevice, boundingBox);

    //================ BEGIN KERNEL ================//
    auto rtcScene = rtcNewScene(mDevice);

    // RTC scene flags
    rtcSetSceneFlags(rtcScene, RTC_SCENE_FLAG_NONE);

    // Selecting higher build quality results in better rendering performance
    // but slower scene commit times. The default build quality for a scene is
    // RTC_BUILD_QUALITY_MEDIUM.
    auto bbquality = RTC_BUILD_QUALITY_HIGH;
    rtcSetSceneBuildQuality(rtcScene, bbquality);

    auto rtcGeometry = mGeometry.getRTCGeometry();
    auto rtcBoundary = mBoundary.getRTCGeometry();

    auto boundaryID = rtcAttachGeometry(rtcScene, rtcBoundary);
    auto geometryID = rtcAttachGeometry(rtcScene, rtcGeometry);
    assert(rtcGetDeviceError(mDevice) == RTC_ERROR_NONE &&
           "Embree device error");

    rtcJoinCommitScene(rtcScene);

    alignas(128) auto rayHit =
        RTCRayHit{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

    auto rtcContext = RTCIntersectContext{};
    rtcInitIntersectContext(&rtcContext);

    auto &baseNodes = baseMesh->getNodes();

    auto &baseNormals = *baseMesh->getPointData().getVectorData(
        lsCalculateNormalVectors<NumericType, D>::normalVectorsLabel);

    std::vector<NumericType> normalThickness(
        baseNodes.size(), std::numeric_limits<NumericType>::infinity());

    // Trace a ray starting from each point of the baseMesh in the normal
    // direction
    for (unsigned i = 0; i < baseNodes.size(); ++i) {
      const auto node = baseNodes[i];
      const auto normal = baseNormals[i];
      auto &ray = rayHit.ray;

#ifdef ARCH_X86
      reinterpret_cast<__m128 &>(ray) =
          _mm_set_ps(1e-4f, (float)node[2], (float)node[1], (float)node[0]);

      reinterpret_cast<__m128 &>(ray.dir_x) = _mm_set_ps(
          0.0f, (float)normal[2], (float)normal[1], (float)normal[0]);
#else
      ray.org_x = (float)node[0];
      ray.org_y = (float)node[1];
      ray.org_z = (float)node[2];
      ray.tnear = 1e-4f;

      ray.dir_x = (float)normal[0];
      ray.dir_y = (float)normal[1];
      ray.dir_z = (float)normal[2];
      ray.time = 0.0f;
#endif

      ray.tfar = std::numeric_limits<rtcNumericType>::max();

      rayHit.hit.instID[0] = RTC_INVALID_GEOMETRY_ID;
      rayHit.hit.geomID = RTC_INVALID_GEOMETRY_ID;

      // Run the intersection
      rtcIntersect1(rtcScene, &rtcContext, &rayHit);

      // if the ray did not hit the geometry, discard it
      if (rayHit.hit.geomID == RTC_INVALID_GEOMETRY_ID ||
          rayHit.hit.geomID == boundaryID) {
        continue;
      }

      normalThickness[i] = ray.tfar;

      // maybe also check for hit from back?

      assert(rayHit.hit.geomID == geometryID && "Geometry hit ID invalid");
    }

    baseMesh->pointData.insertNextScalarData(normalThickness,
                                             "normalThickness");

    rtcReleaseGeometry(rtcGeometry);
    rtcReleaseGeometry(rtcBoundary);

    //================ END KERNEL ================//

    mBoundary.releaseGeometry();
  }

private:
  std::array<std::array<NumericType, 3>, 2> calcBoundingBox() const {
    const auto [xminBase, yminBase, zminBase] = baseMesh->minimumExtent;
    const auto [xmaxBase, ymaxBase, zmaxBase] = baseMesh->maximumExtent;

    const auto [xminSecond, yminSecond, zminSecond] = secondMesh->minimumExtent;
    const auto [xmaxSecond, ymaxSecond, zmaxSecond] = secondMesh->maximumExtent;

    std::array<NumericType, 3> minimumExtent;
    std::array<NumericType, 3> maximumExtent;

    minimumExtent[0] = std::min(xminBase, xminSecond) - mGridDelta;
    minimumExtent[1] = std::min(yminBase, yminSecond) - mGridDelta;
    minimumExtent[2] = std::min(zminBase, zminSecond) - mGridDelta;

    maximumExtent[0] = std::max(xmaxBase, xmaxSecond) + mGridDelta;
    maximumExtent[1] = std::max(ymaxBase, ymaxSecond) + mGridDelta;
    maximumExtent[2] = std::max(zmaxBase, zmaxSecond) + mGridDelta;

    return {minimumExtent, maximumExtent};
  }

  void checkSettings() {

    if (mGeometry.checkGeometryEmpty()) {
      lsMessage::getInstance().addError(
          "No geometry was passed to rayTrace. Aborting.");
    }
  }

  void initMemoryFlags() {
#ifdef ARCH_X86
    // for best performance set FTZ and DAZ flags in MXCSR control and status
    // register
    _MM_SET_FLUSH_ZERO_MODE(_MM_FLUSH_ZERO_ON);
    _MM_SET_DENORMALS_ZERO_MODE(_MM_DENORMALS_ZERO_ON);
#endif
  }
};

#endif