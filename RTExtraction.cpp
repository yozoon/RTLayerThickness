#include <iostream>

#include <lsDomain.hpp>
#include <lsReader.hpp>
#include <lsSmartPointer.hpp>
#include <lsToSurfaceMesh.hpp>
#include <lsVTKWriter.hpp>

#include "rtExtract.hpp"

int main() {
  constexpr int D = 2;
  using NumericType = double;

  auto baseLayer = lsSmartPointer<lsDomain<NumericType, D>>::New();
  lsReader<NumericType, D>(baseLayer, "first_rt.lvst").apply();

  auto depoLayer = lsSmartPointer<lsDomain<NumericType, D>>::New();
  lsReader<NumericType, D>(depoLayer, "second_rt.lvst").apply();

  auto extractor =
      lsSmartPointer<rtExtract<NumericType, D>>::New(baseLayer, depoLayer);
  extractor->apply();
  {
    auto mesh = extractor->getResultingMesh();
    lsVTKWriter<NumericType>(mesh, "substrate.vtp").apply();
  }

  {
    auto mesh = lsSmartPointer<lsMesh<>>::New();
    lsToSurfaceMesh<NumericType, D>(depoLayer, mesh).apply();
    lsVTKWriter<NumericType>(mesh, "depo.vtp").apply();
  }

  return 0;
}