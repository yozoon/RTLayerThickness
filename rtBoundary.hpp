#ifndef RT_BOUNDARY_HPP
#define RT_BOUNDARY_HPP

#include <array>

#include <embree3/rtcore.h>

#include <rayUtil.hpp>

template <typename NumericType, int D> class rtBoundary {
  typedef std::array<std::array<NumericType, 3>, 2> boundingBoxType;

  RTCGeometry mRtcBoundary = nullptr;
  const boundingBoxType mbdBox;

public:
  rtBoundary(RTCDevice &pDevice, const boundingBoxType &pBoundingBox)
      : mbdBox(pBoundingBox) {
    initBoundary(pDevice);
  }

  RTCGeometry &getRTCGeometry() { return mRtcBoundary; }

  void releaseGeometry() {
    // Attention:
    // This function must not be called when the RTCGeometry reference count is
    // > 1 Doing so leads to leaked memory buffers
    if (mTriangleBuffer == nullptr || mVertexBuffer == nullptr ||
        mRtcBoundary == nullptr) {
      return;
    } else {
      rtcReleaseGeometry(mRtcBoundary);
      mRtcBoundary = nullptr;
      mTriangleBuffer = nullptr;
      mVertexBuffer = nullptr;
    }
  }

private:
  void initBoundary(RTCDevice &pDevice) {
    assert(mVertexBuffer == nullptr && mTriangleBuffer == nullptr &&
           "Boundary buffer not empty");

    mRtcBoundary = rtcNewGeometry(pDevice, RTC_GEOMETRY_TYPE_TRIANGLE);

    mVertexBuffer = (vertex_f3_t *)rtcSetNewGeometryBuffer(
        mRtcBoundary, RTC_BUFFER_TYPE_VERTEX,
        0, // the slot
        RTC_FORMAT_FLOAT3, sizeof(vertex_f3_t), numVertices);

    auto xmin = mbdBox[0][0];
    auto xmax = mbdBox[1][0];
    auto ymin = mbdBox[0][1];
    auto ymax = mbdBox[1][1];
    auto zmin = mbdBox[0][2];
    auto zmax = mbdBox[1][2];

    // Vertices
    mVertexBuffer[0].xx = (float)xmin;
    mVertexBuffer[0].yy = (float)ymin;
    mVertexBuffer[0].zz = (float)zmin;

    mVertexBuffer[1].xx = (float)xmax;
    mVertexBuffer[1].yy = (float)ymin;
    mVertexBuffer[1].zz = (float)zmin;

    mVertexBuffer[2].xx = (float)xmax;
    mVertexBuffer[2].yy = (float)ymax;
    mVertexBuffer[2].zz = (float)zmin;

    mVertexBuffer[3].xx = (float)xmin;
    mVertexBuffer[3].yy = (float)ymax;
    mVertexBuffer[3].zz = (float)zmin;

    mVertexBuffer[4].xx = (float)xmin;
    mVertexBuffer[4].yy = (float)ymin;
    mVertexBuffer[4].zz = (float)zmax;

    mVertexBuffer[5].xx = (float)xmax;
    mVertexBuffer[5].yy = (float)ymin;
    mVertexBuffer[5].zz = (float)zmax;

    mVertexBuffer[6].xx = (float)xmax;
    mVertexBuffer[6].yy = (float)ymax;
    mVertexBuffer[6].zz = (float)zmax;

    mVertexBuffer[7].xx = (float)xmin;
    mVertexBuffer[7].yy = (float)ymax;
    mVertexBuffer[7].zz = (float)zmax;

    mTriangleBuffer = (triangle_t *)rtcSetNewGeometryBuffer(
        mRtcBoundary, RTC_BUFFER_TYPE_INDEX,
        0, // slot
        RTC_FORMAT_UINT3, sizeof(triangle_t), numTriangles);

    constexpr rayQuadruple<rayTriple<uint32_t>> xMinMaxPlanes = {
        0, 3, 7, 0, 7, 4, 6, 2, 1, 6, 1, 5};
    constexpr rayQuadruple<rayTriple<uint32_t>> yMinMaxPlanes = {
        0, 4, 5, 0, 5, 1, 6, 7, 3, 6, 3, 2};
    constexpr rayQuadruple<rayTriple<uint32_t>> zMinMaxPlanes = {
        0, 1, 2, 0, 2, 3, 6, 5, 4, 6, 4, 7};

    for (size_t idx = 0; idx < 4; ++idx) {
      mTriangleBuffer[idx].v0 = xMinMaxPlanes[idx][0];
      mTriangleBuffer[idx].v1 = xMinMaxPlanes[idx][1];
      mTriangleBuffer[idx].v2 = xMinMaxPlanes[idx][2];

      mTriangleBuffer[idx + 4].v0 = yMinMaxPlanes[idx][0];
      mTriangleBuffer[idx + 4].v1 = yMinMaxPlanes[idx][1];
      mTriangleBuffer[idx + 4].v2 = yMinMaxPlanes[idx][2];

      mTriangleBuffer[idx + 8].v0 = zMinMaxPlanes[idx][0];
      mTriangleBuffer[idx + 8].v1 = zMinMaxPlanes[idx][1];
      mTriangleBuffer[idx + 8].v2 = zMinMaxPlanes[idx][2];
    }

    rtcCommitGeometry(mRtcBoundary);
    assert(rtcGetDeviceError(pDevice) == RTC_ERROR_NONE &&
           "RTC Error: rtcCommitGeometry");
  }

  struct vertex_f3_t {
    // vertex is the nomenclature of Embree
    // The triangle geometry has a vertex buffer which uses x, y, and z
    // in single precision floating point types.
    float xx, yy, zz;
  };
  vertex_f3_t *mVertexBuffer = nullptr;

  struct triangle_t {
    // The triangle geometry uses an index buffer that contains an array
    // of three 32-bit indices per triangle.
    uint32_t v0, v1, v2;
  };
  triangle_t *mTriangleBuffer = nullptr;

  static constexpr size_t numTriangles = 12;
  static constexpr size_t numVertices = 8;
};

#endif