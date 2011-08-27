//===- l2ptest.cpp - Test for l2p -------------------------------*- C++ -*-===//
//
// L2Package
//
// This file is distributed under the Simplified BSD License. See LICENSE.TXT
// for details.
//
//===----------------------------------------------------------------------===//

#include "l2p/Package.h"
#include "l2p/UObject.h"

#include <iostream>

int main(int argc, char *argv[]) {
  if (argc != 2) {
    std::cerr << "l2ptest l2-root-path\n";
    return 1;
  }

  if (!l2p::Package::Initialize(argv[1])) {
    std::cerr << "Failed to initialize the root dir\n";
    return 1;
  }

  l2p::Package *p = l2p::Package::GetPackage("22_22");
  std::vector<std::shared_ptr<l2p::ATerrainInfo>> terrains;
  p->GetObjects("TerrainInfo", terrains);

  std::shared_ptr<l2p::ATerrainInfo> main = terrains.front();

  std::shared_ptr<l2p::UTexture> t = main->terrain_map;

  return 0;
}
