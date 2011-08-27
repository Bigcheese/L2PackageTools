//===- UObject.cpp - Lineage II client package objects ----------*- C++ -*-===//
//
// L2PackageTools
//
// This file is distributed under the Simplified BSD License. See LICENSE.TXT
// for details.
//
//===----------------------------------------------------------------------===//
//
// This file declares the classes needed to read Lineage II package files.
//
//===----------------------------------------------------------------------===//

#include "l2p/UObject.h"

using namespace l2p;

UObject::~UObject() {}

void UObject::Deserialize(Package &p) {
  if (flags & RF_HasStack) {
    Index node;
    Index dontcare;
    uint64_t dontcaremoar;
    int32_t stilldontcare;
    p >> node
      >> dontcare
      >> Extract<ulittle64_t>(dontcaremoar)
      >> Extract<little32_t>(stilldontcare);
    if (node != 0)
      p >> dontcare;
  }

  if (!(flags & RF_Native)) {
    Property prop;
    do {
      p >> prop;
      SetProperty(prop);
    } while(prop.name != "None");
  }
}
