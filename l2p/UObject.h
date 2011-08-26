//===- UObject.h - Lineage II client package objects ------------*- C++ -*-===//
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

#ifndef L2PACKAGE_UOBJECT_H
#define L2PACKAGE_UOBJECT_H

#include "l2p/Package.h"
#include "l2p/PackedEndian.h"

#include "boost/dynamic_bitset.hpp"

namespace l2p {

struct Property;

class UObject {
public:
  enum Flags {
    RF_Transactional      = 0x00000001,
    RF_Unreachable        = 0x00000002,
    RF_Public             = 0x00000004,
    RF_TagImp             = 0x00000008,
    RF_TagExp             = 0x00000010,
    RF_Obsolete           = 0x00000020,
    RF_TagGarbage         = 0x00000040,
    RF_Final              = 0x00000080,
    RF_PerObjectLocalized = 0x00000100,
    RF_NeedLoad           = 0x00000200,
    RF_HighlightedName    = 0x00000400,
    RF_EliminateObject    = 0x00000400,
    RF_InSingularFunc     = 0x00000800,
    RF_RemappedName       = 0x00000800,
    RF_Suppress           = 0x00001000,
    RF_StateChanged       = 0x00001000,
    RF_InEndState         = 0x00002000,
    RF_Transient          = 0x00004000,
    RF_Preloading         = 0x00008000,
    RF_LoadForClient      = 0x00010000,
    RF_LoadForServer      = 0x00020000,
    RF_LoadForEdit        = 0x00040000,
    RF_Standalone         = 0x00080000,
    RF_NotForClient       = 0x00100000,
    RF_NotForServer       = 0x00200000,
    RF_NotForEdit         = 0x00400000,
    RF_Destroyed          = 0x00800000,
    RF_NeedPostLoad       = 0x01000000,
    RF_HasStack           = 0x02000000,
    RF_Native             = 0x04000000,
    RF_Marked             = 0x08000000,
    RF_ErrorShutdown      = 0x10000000,
    RF_DebugPostLoad      = 0x20000000,
    RF_DebugSerialize     = 0x40000000,
    RF_DebugDestroy       = 0x80000000,
    RF_EdSelected         = 0x80000000,
    RF_ContextFlags       = RF_NotForClient | RF_NotForServer | RF_NotForEdit,
    RF_LoadContextFlags   = RF_LoadForClient | RF_LoadForServer | RF_LoadForEdit,
    RF_Load               = RF_ContextFlags | RF_LoadContextFlags | RF_Public | RF_Final | RF_Standalone | RF_Native | RF_Obsolete | RF_Transactional | RF_HasStack | RF_PerObjectLocalized,
    RF_Keep               = RF_Native | RF_Marked | RF_PerObjectLocalized,
    RF_ScriptMask         = RF_Transactional | RF_Public | RF_Final | RF_Transient | RF_NotForClient | RF_NotForServer | RF_NotForEdit | RF_Standalone
  };

  virtual ~UObject();
  virtual void Deserialize(Package &p);
  // Returns true if consumed.
  virtual bool SetProperty(const Property &p) {
    return false;
  }
  friend Package &operator >>(Package &p, UObject &o) {
    o.Deserialize(p);
    return p;
  }

  uint32_t flags;
  Name name;
  Package *package;
};


struct ArrayIndex {
  int32_t value;

  operator int32_t() const {
    return value;
  }

  friend Package &operator >>(Package &p, ArrayIndex &ai) {
    uint8_t b;
    p >> Extract<ulittle8_t>(b);
    ai.value = b;
    if (b < 128)
      return p;
    if (b & 0x80) {
      p >> Extract<ulittle8_t>(b);
      ai.value |= uint32_t(b) << 8;
    } else if (b & 0xC0) {
      p >> Extract<ulittle8_t>(b);
      ai.value |= uint32_t(b) << 8;
      p >> Extract<ulittle8_t>(b);
      ai.value |= uint32_t(b) << 16;
      p >> Extract<ulittle8_t>(b);
      ai.value |= uint32_t(b) << 24;
    }
    return p;
  }
};

struct Rotator {
  int32_t pitch;
  int32_t yaw;
  int32_t roll;

  friend Package &operator >>(Package &p, Rotator &r) {
    p >> Extract<little32_t>(r.pitch)
      >> Extract<little32_t>(r.yaw)
      >> Extract<little32_t>(r.roll);
    return p;
  }
};

struct Vector {
  float X, Y, Z;
  friend Package &operator >>(Package &p, Vector &v) {
    p >> v.X >> v.Y >> v.Z;
    return p;
  }
};

template<class T>
struct ObjectRef {
  Package *package;
  Index index;
  std::shared_ptr<T> object;

  ObjectRef()
    : package(nullptr)
    , object(nullptr) {
  }

  std::shared_ptr<T> operator ->() {
    return static_cast<std::shared_ptr<T>&>(*this);
  }

  operator std::shared_ptr<T>() {
    if (!object)
      object = std::dynamic_pointer_cast<T>(package->GetObject(index));
    return object;
  }

  friend Package &operator >>(Package &p, ObjectRef &or) {
    or.package = &p;
    p >> or.index;
    return p;
  }
};

struct Property {
  Name name;
  union {
    uint8_t info;
    struct {
      uint8_t type : 4;
      uint8_t size_type : 3;
      uint8_t is_array : 1;
    };
  };
  Name struct_name;
  uint32_t size;
  ArrayIndex array_index;

  union {
    uint8_t uint8_t_value;
    int32_t int32_t_value;
    float float_value;
    Index index_value;
    Vector vector_value;
    Rotator rotator_value;
  };

  friend Package &operator >>(Package &pa, Property &p) {
    pa >> p.name;
    if (p.name == "None")
      return pa;
    pa >> Extract<ulittle8_t>(p.info);
    if (p.type == 10)
      pa >> p.struct_name;
    switch (p.size_type) {
      case 0: p.size = 1; break;
      case 1: p.size = 2; break;
      case 2: p.size = 4; break;
      case 3: p.size = 12; break;
      case 4: p.size = 16; break;
      case 5: pa >> Extract<ulittle8_t>(p.size); break;
      case 6: pa >> Extract<ulittle16_t>(p.size); break;
      case 7: pa >> Extract<ulittle32_t>(p.size); break;
    }

    if (p.type == 3) // Boolean.
      return pa;
    if (p.is_array) {
      pa >> p.array_index;
    }

    switch (p.type) {
    case 0x01:
      pa >> Extract<ulittle8_t>(p.uint8_t_value);
      break;
    case 0x02:
      pa >> Extract<little32_t>(p.int32_t_value);
      break;
    case 0x04:
      pa >> p.float_value;
      break;
    case 0x05:
    case 0x06:
      pa >> p.index_value;
      break;
    case 0x0A:
      if (p.struct_name == "Vector")
        pa >> p.vector_value;
      else if (p.struct_name == "Rotator")
        pa >> p.rotator_value;
      else
        static_cast<std::istream&>(pa).seekg(p.size, std::ios::cur);
      break;
    case 0x0C:
      pa >> p.rotator_value;
      break;
    default:
      static_cast<std::istream&>(pa).seekg(p.size, std::ios::cur);
    }

    return pa;
  }
};

struct Box {
  Vector min;
  Vector max;
  uint8_t is_valid;

  Box() : is_valid(0) {}

  friend Package &operator >>(Package &p, Box &b) {
    p >> b.min
      >> b.max
      >> Extract<ulittle8_t>(b.is_valid);
    return p;
  }

  Box &operator +=(const Vector &v) {
    if (is_valid) {
      min.X = std::min(min.X, v.X);
      min.Y = std::min(min.Y, v.Y);
      min.Z = std::min(min.Z, v.Z);

      max.X = std::max(max.X, v.X);
      max.Y = std::max(max.Y, v.Y);
      max.Z = std::max(max.Z, v.Z);
    } else {
      min = max = v;
      is_valid = 1;
    }
    return *this;
  }
};

struct Sphere {
  Vector location;
  float radius;

  friend Package &operator >>(Package &p, Sphere &s) {
    p >> s.location >> s.radius;
    return p;
  }
};

struct Plane {
  float X, Y, Z, W;

  friend Package &operator >>(Package &p, Plane &pl) {
    p >> pl.X >> pl.Y >> pl.Z >> pl.W;
    return p;
  }
};

class UPrimative : public UObject {
public:
  Box bounding_box;
  Sphere bounding_sphere;

  void Deserialize(Package &p) {
    UObject::Deserialize(p);
    p >> bounding_box >> bounding_sphere;
  }
};

struct BSPNode {
  Plane plane;
  uint64_t zone_mask;
  uint8_t node_flags;
  Index vert_pool;
  Index surface;
  Index back;
  Index front;
  Index i_plane; // Nexy coplanar poly.
  Index collision_bound;
  Index render_bound;
  Vector unknown_point;
  uint32_t unknown_id;
  uint64_t conn_zones;
  uint64_t vis_zones;
  Index zone[2];
  uint8_t num_verticies;
  int32_t leaf[2];

  friend Package &operator >>(Package &p, BSPNode &n) {
    p >> n.plane
      >> Extract<ulittle64_t>(n.zone_mask)
      >> Extract<ulittle8_t>(n.node_flags)
      >> n.vert_pool
      >> n.surface
      >> n.back
      >> n.front
      >> n.i_plane
      >> n.collision_bound
      >> n.render_bound
      >> n.unknown_point
      >> Extract<ulittle32_t>(n.unknown_id)
      >> Extract<ulittle64_t>(n.conn_zones)
      >> Extract<ulittle64_t>(n.vis_zones)
      >> n.zone[0]
      >> n.zone[1]
      >> Extract<ulittle8_t>(n.num_verticies)
      >> Extract<little32_t>(n.leaf[0])
      >> Extract<little32_t>(n.leaf[1]);
    char unk[0xC];
    static_cast<std::istream&>(p).read(unk, sizeof(unk));
    return p;
  }
};

enum EPolyFlags
{
  PF_Invisible  = 0x00000001,
  PF_NotSolid   = 0x00000008,
  PF_Semisolid  = 0x00000020,
  PF_GeomMarked = 0x00000040,
  PF_TwoSided   = 0x00000100,
  PF_Portal     = 0x04000000,
  PF_Mirrored   = 0x20000000,

  PF_WeDontCareAbout = PF_Invisible | PF_NotSolid | PF_Portal
};

struct BSPSurface {
  ObjectRef<UObject> material;
  uint32_t flags;
  Index base;
  Index normal;
  Index U;
  Index V;
  ObjectRef<UObject> brush_poly;
  ObjectRef<UObject> actor;
  Plane plane;
  uint32_t unk[2];

  friend Package &operator >>(Package &p, BSPSurface &s) {
    p >> s.material
      >> Extract<ulittle32_t>(s.flags)
      >> s.base
      >> s.normal
      >> s.U
      >> s.V
      >> s.brush_poly
      >> s.actor
      >> s.plane
      >> Extract<ulittle32_t>(s.unk[0])
      >> Extract<ulittle32_t>(s.unk[1]);
    return p;
  }
};

struct Vertex {
  Index vertex;
  Index side;

  friend Package &operator >>(Package &p, Vertex &v) {
    p >> v.vertex
      >> v.side;
    return p;
  }
};

class UModel : public UPrimative {
public:
  std::vector<Vector> vectors;
  std::vector<Vector> points;
  std::vector<BSPNode> nodes;
  std::vector<BSPSurface> surfaces;
  std::vector<Vertex> vertexes;

  void Deserialize(Package &p) {
    UPrimative::Deserialize(p);

    p >> ExtractArray<Index, Vector>(vectors)
      >> ExtractArray<Index, Vector>(points)
      >> ExtractArray<Index, BSPNode>(nodes)
      >> ExtractArray<Index, BSPSurface>(surfaces)
      >> ExtractArray<Index, Vertex>(vertexes);
  }

  Box getRegionAABB() const {
    int32_t minX = (regionX - 20) * 32768;
    int32_t minY = (regionY - 18) * 32768;
    Box b;
    b.min.X = static_cast<float>(minX);
    b.min.Y = static_cast<float>(minY);
    b.min.Z = -16383.f;
    b.max.X = static_cast<float>(minX + 32768);
    b.max.Y = static_cast<float>(minY + 32768);
    b.max.Z = 16383.f;
    b.is_valid = 1;
    return b;
  }

  // The Lineage II map region number.
  int32_t regionX;
  int32_t regionY;
};

class AActor : public UObject {
public:
  Vector location;
  Rotator rotation;
  float draw_scale;

  virtual bool SetProperty(const Property &p) {
    if (UObject::SetProperty(p))
      return true;

    if (p.name == "Location") {
      location = p.vector_value;
      return true;
    } else if (p.name == "Rotation") {
      rotation = p.rotator_value;
      return true;
    } else if (p.name == "DrawScale") {
      draw_scale = p.float_value;
      return true;
    }
    return false;
  }
};

class AInfo : public AActor {};

class ATerrainInfo : public AInfo {
public:
  Index terrain_map;
  Vector terrain_scale;
  boost::dynamic_bitset<> quad_visibility_bitmap;
  boost::dynamic_bitset<> edge_turn_bitmap;
  int32_t map_x;
  int32_t map_y;

  virtual bool SetProperty(const Property &p) {
    if (AInfo::SetProperty(p))
      return true;

    if (p.name == "TerrainMap") {
      terrain_map = p.index_value;
      return true;
    } else if (p.name == "TerrainScale") {
      terrain_scale = p.vector_value;
      return true;
    } else if (p.name == "QuadVisibilityBitmap") {
      return true;
    } else if (p.name == "EdgeTurnBitmap") {
      return true;
    } else if (p.name == "MapX") {
      map_x = p.int32_t_value;
      return true;
    } else if (p.name == "MapY") {
      map_y = p.int32_t_value;
      return true;
    }

    return false;
  }
};

}

#endif
