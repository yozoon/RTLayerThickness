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

  lsVTKWriter<NumericType>(extractor->getBaseMesh(), "substrate.vtp").apply();

  lsVTKWriter<NumericType>(extractor->getSecondMesh(), "depo.vtp").apply();

  return 0;
}