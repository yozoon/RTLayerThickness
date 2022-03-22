#include <iostream>

#include <lsDomain.hpp>
#include <lsReader.hpp>
#include <lsSmartPointer.hpp>
#include <lsVTKWriter.hpp>

#include "rtExtract.hpp"

int main() {
  constexpr int D = 2;
  using NumericType = double;

  auto baseLayer = lsSmartPointer<lsDomain<NumericType, D>>::New();
  lsReader<NumericType, D>(baseLayer, "first.lvst").apply();

  auto depoLayer = lsSmartPointer<lsDomain<NumericType, D>>::New();
  lsReader<NumericType, D>(depoLayer, "second.lvst").apply();

  auto extractor =
      lsSmartPointer<rtExtract<NumericType, D>>::New(baseLayer, depoLayer);

  extractor->apply();

  auto mesh = extractor->getResultingMesh();

  lsVTKWriter<NumericType>(mesh, "result.vtp").apply();

  return 0;
}