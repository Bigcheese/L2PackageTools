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

#include <unordered_map>

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

  std::string GetPath() {
    return package->name.str() + "." + name.str();
  }

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

struct Color {
  uint8_t b, g, r, a;

  friend Package &operator >>(Package &p, Color &c) {
    p >> Extract<ulittle8_t>(c.b)
      >> Extract<ulittle8_t>(c.g)
      >> Extract<ulittle8_t>(c.r)
      >> Extract<ulittle8_t>(c.a);
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
    index.value = 0;
  }

  std::shared_ptr<T> operator ->() {
    if (!object && index)
      object = std::dynamic_pointer_cast<T>(package->GetObject(index));
    return object;
  }

  operator std::shared_ptr<T>() {
    if (!object && index)
      object = std::dynamic_pointer_cast<T>(package->GetObject(index));
    return object;
  }

  friend Package &operator >>(Package &p, ObjectRef &or) {
    or.package = &p;
    p >> or.index;
    return p;
  }
};

enum ESheerAxis {
  SHEER_None = 0,
  SHEER_XY   = 1,
  SHEER_XZ   = 2,
  SHEER_YX   = 3,
  SHEER_YZ   = 4,
  SHEER_ZX   = 5,
  SHEER_ZY   = 6,
};

struct Scale {
  Vector scale;
  float sheer_rate;
  ESheerAxis sheer_axis;

  friend Package &operator >>(Package &p, Scale &s) {
    p >> s.scale
      >> s.sheer_rate
      >> Extract<ulittle8_t>(s.sheer_rate);
    return p;
  }
};

struct Matrix {
  float m[16];
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
  int32_t array_size;

  union {
    uint8_t uint8_t_value;
    int32_t int32_t_value;
    float float_value;
    Index index_value;
    Vector vector_value;
    Rotator rotator_value;
    Scale scale_value;
  };
  std::vector<uint8_t> data_value;
  std::vector<std::unordered_map<std::string, Property>> property_list;

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
    case 0x09: {
        std::streampos a = static_cast<std::istream&>(pa).tellg();
        pa >> Extract<Index>(p.array_size);
        std::streamoff asize = static_cast<std::istream&>(pa).tellg() - a;

        // HACK: This information actually has to be passed by the
        //       UClass/UStruct so that we know how to interpret the data.
        if (p.name == "Materials") {
          p.property_list.reserve(p.array_size);
          std::streampos prop_array_start = static_cast<std::istream&>(pa).tellg();
          for (int i = 0; i < p.array_size; ++i) {
            std::unordered_map<std::string, Property> pmap;
            while (true) {
              Property ap;
              pa >> ap;
              if (ap.name == "None")
                break;
              pmap[ap.name] = std::move(ap);
            }
            p.property_list.push_back(std::move(pmap));
          }
          std::streampos prop_array_end = static_cast<std::istream&>(pa).tellg();
          assert((prop_array_end - prop_array_start) == p.size - asize &&
                 "Failed to properly deserialize property of type array!");
        } else {
          p.data_value.resize(std::size_t(p.size - asize));
          static_cast<std::istream&>(pa).read((char*)&p.data_value.front(), p.size - asize);
        }
      }
      break;
    case 0x0A:
      if (p.struct_name == "Vector")
        pa >> p.vector_value;
      else if (p.struct_name == "Rotator")
        pa >> p.rotator_value;
      else if (p.struct_name == "TerrainLayer") {
        std::unordered_map<std::string, Property> pmap;
        while (true) {
          Property ap;
          pa >> ap;
          if (ap.name == "None")
            break;
          pmap[ap.name] = std::move(ap);
        }
        p.property_list.push_back(std::move(pmap));
      } else
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

class UMaterial : public UObject {
public:
  ObjectRef<UMaterial> FallbackMaterial;
  ObjectRef<UMaterial> DefaultMaterial;
};

class UModifier : public UMaterial {
public:
  ObjectRef<UMaterial> Material;

  virtual bool SetProperty(const Property &p) {
    if (UMaterial::SetProperty(p))
      return true;

    if (p.name == "Material") {
      Material.index = p.index_value;
      Material.package = package;
      return true;
    }

    return false;
  }
};

class UFinalBlend : public UModifier {
public:
  enum EFrameBufferBlending {
    FB_Overwrite,
    FB_Modulate,
    FB_AlphaBlend,
    FB_AlphaModulate_MightNotFogCorrectly,
    FB_Translucent,
    FB_Darken,
    FB_Brighten,
    FB_Invisible,
    FB_Add,
    FB_InWaterBlend,
    FB_Capture,
    FB_OverwriteAlpha,
    FB_DarkenInv
  } FrameBufferBlending;
  bool ZWrite;
  bool ZTest;
  bool AlphaTest;
  bool TwoSided;
  uint8_t AlphaRef;
  bool TreatAsTwoSided;

  UFinalBlend()
    : FrameBufferBlending(FB_Overwrite)
    , ZWrite(true)
    , ZTest(true)
    , AlphaTest(false)
    , TwoSided(false)
    , AlphaRef(128)
    , TreatAsTwoSided(false) {
  }

  virtual bool SetProperty(const Property &p) {
    if (UModifier::SetProperty(p))
      return true;

    if (p.name == "FrameBufferBlending") {
      FrameBufferBlending = static_cast<EFrameBufferBlending>(p.uint8_t_value);
      return true;
    } else if (p.name == "ZWrite") {
      ZWrite = p.is_array;
      return true;
    } else if (p.name == "ZTest") {
      ZTest = p.is_array;
      return true;
    } else if (p.name == "AlphaTest") {
      AlphaTest = p.is_array;
      return true;
    } else if (p.name == "TwoSided") {
      TwoSided = p.is_array;
      return true;
    } else if (p.name == "AlphaRef") {
      AlphaRef = p.uint8_t_value;
      return true;
    } else if (p.name == "TreatAsTwoSided") {
      TreatAsTwoSided = p.is_array;
      return true;
    }

    return false;
  }
};

class URenderedMaterial : public UMaterial {};

class UBitmapMaterial : public URenderedMaterial {
public:
  enum ETextureFormat {
    TEXF_P8,
    TEXF_RGBA7,
    TEXF_RGB16,
    TEXF_DXT1,
    TEXF_RGB8,
    TEXF_RGBA8,
    TEXF_NODATA,
    TEXF_DXT3,
    TEXF_DXT5,
    TEXF_L8,
    TEXF_G16,
    TEXF_RRRGGGBBB,
  } Format;
  uint8_t UBits, VBits;
  int32_t USize, VSize;
  int32_t UClamp, VClamp;
  int32_t LossDetail;
  int32_t MinFilter;
  int32_t MagFilter;
  int32_t MipFilter;

  enum ETexClampMode {
    TC_Wrap,
    TC_Clamp
  } UClampMode, VClampMode;

  virtual bool SetProperty(const Property &p) {
    if (URenderedMaterial::SetProperty(p))
      return true;

    if (p.name == "Format") {
      Format = static_cast<ETextureFormat>(p.uint8_t_value);
      return true;
    } else if (p.name == "UBits") {
      UBits = p.uint8_t_value;
      return true;
    } else if (p.name == "VBits") {
      VBits = p.uint8_t_value;
      return true;
    } else if (p.name == "USize") {
      USize = p.int32_t_value;
      return true;
    } else if (p.name == "VSize") {
      VSize = p.int32_t_value;
      return true;
    } else if (p.name == "UClamp") {
      UClamp = p.int32_t_value;
      return true;
    } else if (p.name == "VClamp") {
      VClamp = p.int32_t_value;
      return true;
    }

    return false;
  }
};

struct MIPS {
  int32_t unknown;
  std::vector<uint8_t> data;
  int32_t USize;
  int32_t VSize;
  int8_t UBits;
  int8_t VBits;

  // Helper to access data.
  template<class T>
  T *getAs() {
    return reinterpret_cast<T*>(&data.front());
  }

  friend Package &operator >>(Package &p, MIPS &m) {
    p >> Extract<little32_t>(m.unknown)
      >> ExtractArray<Index, ulittle8_t>(m.data)
      >> Extract<little32_t>(m.USize)
      >> Extract<little32_t>(m.VSize)
      >> Extract<little8_t>(m.UBits)
      >> Extract<little8_t>(m.VBits);
    return p;
  }
};

class UTexture : public UBitmapMaterial {
public:
  enum ELODSet {
    LODSET_None,
    LODSET_World,
    LODSET_PlayerSkin,
    LODSET_WeaponSkin,
    LODSET_Terrain,
    LODSET_Interface,
    LODSET_RenderMap,
    LODSET_Lightmap
  } LODSet;

  ObjectRef<UMaterial> Detail;
  float DetailScale;
  Color MipZero;
  Color MaxColor;
  int InternalTime[2];
  bool bMasked;
  bool bAlphaTexture;
  bool bTwoSided;
  bool bHighColorQuality;
  bool bHighTextureQuality;
  bool bRealtime;
  bool bParametric;
  bool bHasComp;
  bool bSplit9Texture;
  int Split9X1;
  int Split9X2;
  int Split9X3;
  int Split9Y1;
  int Split9Y2;
  int Split9Y3;
  int NormalLOD;
  int MinLOD;
  ObjectRef<UTexture> AnimNext;
  uint8_t PrimeCount;
  float MinFrameRate;
  float MaxFrameRate;
  int TotalFrameNum;
  bool OneTimeAnimLoop;
  std::vector<MIPS> mips;
  ETextureFormat CompFormat;

  UTexture()
    : DetailScale(8.f)
    , LODSet(LODSET_World)
    , bAlphaTexture(false)
    , bTwoSided(false) {
    MipZero.r = 64;
    MipZero.g = 128;
    MipZero.b = 64;
    MipZero.a = 0;
    MaxColor.r = MaxColor.g = MaxColor.b = MaxColor.a = 255;
  }

  virtual bool SetProperty(const Property &p) {
    if (UBitmapMaterial::SetProperty(p))
      return true;

    if (p.name == "bAlphaTexture") {
      bAlphaTexture = p.is_array;
      return true;
    } else if (p.name == "bTwoSided") {
      bTwoSided = p.is_array;
      return true;
    }

    return false;
  }

  virtual void Deserialize(Package &p) {
    UBitmapMaterial::Deserialize(p);

    // Read a bunch of stuff I don't understand.
    char giant_buffer_of_dont_care[4096];
    static_cast<std::istream&>(p).read(giant_buffer_of_dont_care, 36);
    Index dontcare;
    p >> dontcare;
    if (dontcare == 0)
      return;
    p >> dontcare;
    static_cast<std::istream&>(p).read(giant_buffer_of_dont_care, dontcare);
    static_cast<std::istream&>(p).read(giant_buffer_of_dont_care, 1);
    p >> dontcare;
    static_cast<std::istream&>(p).read(giant_buffer_of_dont_care, dontcare);
    static_cast<std::istream&>(p).read(giant_buffer_of_dont_care, 4);

    p >> ExtractArray<Index, MIPS>(mips);
  }
};

class UShader : public URenderedMaterial {
public:
  enum EOutputBlending {
    OB_Normal,
    OB_Masked,
    OB_Modulate,
    OB_Translucent,
    OB_Invisible,
    OB_Brighten,
    OB_Darken,
  } OutputBlending;

  ObjectRef<UTexture> Diffuse;
  ObjectRef<UTexture> Opacity;
  ObjectRef<UTexture> Specular;
  ObjectRef<UTexture> SpecularMask;
  ObjectRef<UTexture> SelfIllumination;
  ObjectRef<UTexture> SelfIlluminationMask;
  ObjectRef<UTexture> Detail;
  float DetailScale;
  bool TwoSided;
  bool Wireframe;
  bool ModulateStaticLighting2X;
  bool PerformLightingOnSpecularPass;
  bool TreatAsTwoSided;
  bool ZWrite;
  bool AlphaTest;
  uint8_t AlphaRef;
  ObjectRef<UTexture> NormalMap;
  float BumpOffsetScaleFactor;
  float BumpOffsetBiasFactor;
  ObjectRef<UTexture> SpecularMap;
  float SpecularPower;
  float SpecularScale;

  UShader()
    : DetailScale(8.f)
    , ZWrite(true)
    , SpecularPower(15.f)
    , SpecularScale(1.f)
    , AlphaTest(false)
    , AlphaRef(128) {
  }

  virtual bool SetProperty(const Property &p) {
    if (URenderedMaterial::SetProperty(p))
      return true;

    if (p.name == "Diffuse") {
      Diffuse.index = p.index_value;
      Diffuse.package = package;
      return true;
    } else if (p.name == "OutputBlending") {
      OutputBlending = static_cast<EOutputBlending>(p.uint8_t_value);
      return true;
    } else if (p.name == "AlphaTest") {
      AlphaTest = p.is_array;
      return true;
    } else if (p.name == "AlphaRef") {
      AlphaRef = p.uint8_t_value;
      return true;
    } else if (p.name == "TreatAsTwoSided") {
      TreatAsTwoSided = p.is_array;
      return true;
    } else if (p.name == "TwoSided") {
      TwoSided = p.is_array;
      return true;
    }

    return false;
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
  Index i_plane; // Next coplanar poly.
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
  ObjectRef<UMaterial> material;
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

struct SMeshSurface {
  uint32_t unknown;
  uint16_t index_offset;
  uint16_t unknown01;
  uint16_t vertex_max;
  uint16_t triangle_count;
  uint16_t triangle_max;

  friend Package &operator >>(Package &p, SMeshSurface &sms) {
    p >> Extract<ulittle32_t>(sms.unknown)
      >> Extract<ulittle16_t>(sms.index_offset)
      >> Extract<ulittle16_t>(sms.unknown01)
      >> Extract<ulittle16_t>(sms.vertex_max)
      >> Extract<ulittle16_t>(sms.triangle_count)
      >> Extract<ulittle16_t>(sms.triangle_max);
    return p;
  }
};

struct SMeshVertex {
  Vector location, normal;

  friend Package &operator >>(Package &p, SMeshVertex &smv) {
    p >> smv.location
      >> smv.normal;
    return p;
  }
};

struct SMeshCoord {
  float u, v;

  friend Package &operator >>(Package &p, SMeshCoord &smc) {
    p >> smc.u
      >> smc.v;
    return p;
  }
};

struct SMeshCoords {
  std::vector<SMeshCoord> elements;
  uint32_t unk1;
  uint32_t unk2;

  friend Package &operator >>(Package &p, SMeshCoords &smc) {
    p >> ExtractArray<Index, SMeshCoord>(smc.elements)
      >> Extract<ulittle32_t>(smc.unk1)
      >> Extract<ulittle32_t>(smc.unk2);
    return p;
  }
};

struct SMeshMaterial {
  bool enable_collision;
  ObjectRef<UMaterial> material;
};

class UStaticMesh : public UPrimative {
public:
  // Native serialized data.
  std::vector<SMeshSurface> surfaces;
  Box another_bb;
  std::vector<SMeshVertex> vertices;
  std::vector<Color> vertex_colors_1;
  std::vector<Color> vertex_colors_2;
  std::vector<SMeshCoords> texture_coords;
  std::vector<uint16_t> vertex_indicies_1;
  std::vector<uint16_t> vertex_indicies_2;

  // Properties.
  std::vector<SMeshMaterial> materials;
  bool UseSimpleLineCollision;
  bool UseSimpleBoxCollision;
  bool UseSimpleKarmaCollision;

  UStaticMesh()
    : UseSimpleLineCollision(false)
    , UseSimpleBoxCollision(true)
    , UseSimpleKarmaCollision(true) {
  }

  virtual bool SetProperty(const Property &p) {
    if (UPrimative::SetProperty(p))
      return true;

    if (p.name == "Materials") {
      for (int i = 0; i < p.array_size; ++i) {
        SMeshMaterial smm;
        auto itter = p.property_list[i].find("EnableCollision");
        if (itter != p.property_list[i].end())
          smm.enable_collision = itter->second.is_array;
        else
          smm.enable_collision = false;
        itter = p.property_list[i].find("Material");
        if (itter != p.property_list[i].end()) {
          smm.material.index = itter->second.index_value;
          smm.material.package = package;
        }
        materials.push_back(std::move(smm));
      }
      return true;
    } else if (p.name == "UseSimpleLineCollision") {
      UseSimpleLineCollision = p.is_array;
      return true;
    } else if (p.name == "UseSimpleBoxCollision") {
      UseSimpleBoxCollision = p.is_array;
      return true;
    } else if (p.name == "UseSimpleKarmaCollision") {
      UseSimpleKarmaCollision = p.is_array;
      return true;
    }

    return false;
  }

  void Deserialize(Package &p) {
    UPrimative::Deserialize(p);

    uint32_t unk;

      p >> ExtractArray<Index, SMeshSurface>(surfaces)
        >> another_bb
        >> ExtractArray<Index, SMeshVertex>(vertices)
        >> Extract<ulittle32_t>(unk)
        >> ExtractArray<Index, Color>(vertex_colors_1)
        >> Extract<ulittle32_t>(unk)
        >> ExtractArray<Index, Color>(vertex_colors_2)
        >> Extract<ulittle32_t>(unk)
        >> ExtractArray<Index, SMeshCoords>(texture_coords)
        >> ExtractArray<Index, ulittle16_t>(vertex_indicies_1)
        >> Extract<ulittle32_t>(unk)
        >> ExtractArray<Index, ulittle16_t>(vertex_indicies_2)
        >> Extract<ulittle32_t>(unk);
  }
};

class AActor : public UObject {
public:
  Vector location;
  Rotator rotation;
  float draw_scale;
  Vector draw_scale_3d;
  Vector pre_pivot;
  ObjectRef<UStaticMesh> static_mesh;
  unsigned bDeleteMe : 1;
  unsigned bHidden : 1;
  unsigned bCollideActors : 1;
  unsigned bBlockActors : 1;
  unsigned bBlockPlayers : 1;

  AActor()
    : draw_scale(1.f)
    , bDeleteMe(false)
    , bHidden(false)
    , bCollideActors(true)
    , bBlockActors(true)
    , bBlockPlayers(true) {
    draw_scale_3d.X = 1.f;
    draw_scale_3d.Y = 1.f;
    draw_scale_3d.Z = 1.f;
    rotation.pitch = 0;
    rotation.yaw = 0;
    rotation.roll = 0;
    pre_pivot.X = 0.f;
    pre_pivot.Y = 0.f;
    pre_pivot.Z = 0.f;
  }

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
    } else if (p.name == "DrawScale3D") {
      draw_scale_3d = p.vector_value;
      return true;
    } else if (p.name == "StaticMesh") {
      static_mesh.index = p.index_value;
      static_mesh.package = package;
      return true;
    } else if (p.name == "bDeleteMe") {
      bDeleteMe = p.is_array;
      return true;
    } else if (p.name == "bHidden") {
      bHidden = p.is_array;
      return true;
    } else if (p.name == "bCollideActors") {
      bCollideActors = p.is_array;
      return true;
    } else if (p.name == "bBlockActors") {
      bBlockActors = p.is_array;
      return true;
    } else if (p.name == "bBlockPlayers") {
      bBlockPlayers = p.is_array;
      return true;
    } else if (p.name == "PrePivot") {
      pre_pivot = p.vector_value;
      return true;
    }
    return false;
  }
};

class AStaticMeshActor : public AActor {};

class AInfo : public AActor {};

struct TerrainLayer {
  ObjectRef<UTexture> Texture;
  ObjectRef<UTexture> AlphaMap;
  float UScale;
  float VScale;
  float UPan;
  float VPan;
  enum ETextureMapAxis {
    TEXMAPAXIS_XY,
    TEXMAPAXIS_XZ,
    TEXMAPAXIS_YZ,
  } TextureMapAxis;
  float TextureRotation;
  Rotator LayerRotation;
  Matrix TerrainMatrix;
  float KFriction;
  float Krestitution;
  ObjectRef<UTexture> LayerWeightMap;
  Vector Scale_;
  Vector ToWorld[4];
  Vector ToMaskmap[4];
  bool bUseAlpha;
};

class ATerrainInfo : public AInfo {
public:
  ObjectRef<UTexture> terrain_map;
  Vector terrain_scale;
  boost::dynamic_bitset<uint8_t> quad_visibility_bitmap;
  boost::dynamic_bitset<uint8_t> edge_turn_bitmap;
  int32_t map_x;
  int32_t map_y;
  std::vector<TerrainLayer> Layers;

  virtual bool SetProperty(const Property &p) {
    if (AInfo::SetProperty(p))
      return true;

    if (p.name == "TerrainMap") {
      terrain_map.index = p.index_value;
      terrain_map.package = package;
      return true;
    } else if (p.name == "TerrainScale") {
      terrain_scale = p.vector_value;
      return true;
    } else if (p.name == "QuadVisibilityBitmap") {
      quad_visibility_bitmap.append(p.data_value.begin(), p.data_value.end());
      return true;
    } else if (p.name == "EdgeTurnBitmap") {
      edge_turn_bitmap.append(p.data_value.begin(), p.data_value.end());
      return true;
    } else if (p.name == "MapX") {
      map_x = p.int32_t_value;
      return true;
    } else if (p.name == "MapY") {
      map_y = p.int32_t_value;
      return true;
    } else if (p.name == "Layers") {
      TerrainLayer tl;
      auto itter = p.property_list[0].find("Texture");
      if (itter != p.property_list[0].end()) {
        tl.Texture.index = itter->second.index_value;
        tl.Texture.package = package;
      }
      itter = p.property_list[0].find("AlphaMap");
      if (itter != p.property_list[0].end()) {
        tl.AlphaMap.index = itter->second.index_value;
        tl.AlphaMap.package = package;
      }
      itter = p.property_list[0].find("UScale");
      if (itter != p.property_list[0].end())
        tl.UScale = itter->second.float_value;
      itter = p.property_list[0].find("VScale");
      if (itter != p.property_list[0].end())
        tl.VScale = itter->second.float_value;
      itter = p.property_list[0].find("UPan");
      if (itter != p.property_list[0].end())
        tl.UPan = itter->second.float_value;
      itter = p.property_list[0].find("VPan");
      if (itter != p.property_list[0].end())
        tl.VPan = itter->second.float_value;
      itter = p.property_list[0].find("TextureMapAxis");
      if (itter != p.property_list[0].end())
        tl.TextureMapAxis = static_cast<TerrainLayer::ETextureMapAxis>(itter->second.uint8_t_value);
      itter = p.property_list[0].find("TextureRotation");
      if (itter != p.property_list[0].end())
        tl.TextureRotation = itter->second.float_value;
      Layers.push_back(std::move(tl));
      return true;
    }

    return false;
  }
};

enum ECSGOperation {
  CSG_Active      = 0,
  CSG_Add         = 1,
  CSG_Subtract    = 2,
  CSG_Intersect   = 3,
  CSG_Deintersect = 4,
  CSG_MAX         = 5,
};

class ABrush : public AActor {
public:
  ECSGOperation csg_operation;
  Color color;
  int32_t poly_flags;
  bool bColored;
  ObjectRef<UModel> brush;

  virtual bool SetProperty(const Property &p) {
    if (AActor::SetProperty(p))
      return true;

    if (p.name == "Brush") {
      brush.index = p.index_value;
      brush.package = package;
    }

    return false;
  }
};

class AVolume : public ABrush {};
class ABlockingVolume : public AVolume {};

class APhysicsVolume : public AVolume {
public:
  bool bWaterVolume;

  APhysicsVolume() : bWaterVolume(false) {}
};

class AWaterVolume : public APhysicsVolume {
public:
  AWaterVolume() {
    bWaterVolume = true;
  }
};

}

#endif
