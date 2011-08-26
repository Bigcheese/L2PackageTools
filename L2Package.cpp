//===- L2Package.cpp - Lineage II client package reader ---------*- C++ -*-===//
//
// L2Package
//
// This file is distributed under the Simplified BSD License. See LICENSE.TXT
// for details.
//
//===----------------------------------------------------------------------===//
//
// This file declares the classes needed to read Lineage II package files.
//
//===----------------------------------------------------------------------===//

#include "L2Package.h"

using namespace L2Package;

std::map<std::string, Archive*> L2Package::packages;
std::string L2Path;
