//===- l2ptest.cpp - Test for l2p -------------------------------*- C++ -*-===//
//
// L2PackageTools
//
// This file is distributed under the Simplified BSD License. See LICENSE.TXT
// for details.
//
//===----------------------------------------------------------------------===//

#include "l2p/Package.h"
#include "l2p/UObject.h"

#include "boost/program_options.hpp"

#include <iostream>

int main(int argc, char *argv[]) {
  if (argc != 3) {
    std::cerr << "l2ptest l2-root-path package-name\n";
    return 1;
  }

  if (!l2p::Package::Initialize(argv[1])) {
    std::cerr << "Failed to initialize the root dir\n";
    return 1;
  }

  l2p::Package *p = l2p::Package::GetPackage(argv[2]);
  std::vector<std::shared_ptr<l2p::UStaticMesh>> smeshes;
  p->GetObjects("StaticMesh", smeshes);

  for (auto i = smeshes.begin(), e = smeshes.end(); i != e; ++i) {
    if ((*i)->UseSimpleBoxCollision) {
      std::cout << (*i)->name.str() << "\n";
    }
  }

  return 0;
}
