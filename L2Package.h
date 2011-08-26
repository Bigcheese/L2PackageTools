//===- L2Package.h - Lineage II client package reader -----------*- C++ -*-===//
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

#ifndef L2PACKAGE_H
#define L2PACKAGE_H

#include "PackedEndian.h"
#include "StringRef.h"

#include "boost/dynamic_bitset.hpp"

#include "Ogre.h"

#include <array>
#include <cassert>
#include <cstdint>
#include <exception>
#include <fstream>
#include <streambuf>
#include <unordered_map>
#include <vector>

namespace L2Package {
using std::uint64_t;
using std::uint32_t;
using std::int32_t;
using std::int16_t;
using std::uint8_t;

class l2_decrypt_streambuf : public std::basic_streambuf<char>
{
public:
  typedef std::basic_streambuf<char>::traits_type traits_type;
  typedef traits_type::int_type int_type;
  typedef traits_type::pos_type pos_type;
  typedef traits_type::off_type off_type;

  explicit l2_decrypt_streambuf(std::basic_streambuf<char> &orig)
    : orig_streambuf(orig)
    , buffer(BUFF_SIZE) {
      // Setup the input buffer so that underflow is called on the first read.
      setg(0, 0, 0);


      std::array<char, 22> target_header = {
        0x4C, 0x00, 0x69, 0x00, 0x6E, 0x00, 0x65, 0x00,
        0x61, 0x00, 0x67, 0x00, 0x65, 0x00, 0x32, 0x00,
        0x56, 0x00, 0x65, 0x00, 0x72, 0x00
      };
      // Read the version from orig_streambuf.
      std::array<char, 22> header_buf;
      std::array<char, 6>  version_buf;
      orig_streambuf.sgetn(header_buf.data(), 22);
      orig_streambuf.sgetn(version_buf.data(), 6);
      if (header_buf != target_header) {
        buffer.resize(0);
        return;
      }

      // Get the version as an integer.
      std::string ver;
      ver += version_buf[0];
      ver += version_buf[2];
      ver += version_buf[4];

      std::istringstream out(ver);
      int archive_version;
      out >> archive_version;
      if (!out) {
        buffer.resize(0);
        return;
      }

      // Get key from version.
      switch (archive_version) {
      case 111: key = 0xAC; break;
      case 121: key = 0x89; break;
      default:
        buffer.resize(0);
        return;
      }

      // Get the total file size so we know when to return EOF.
      file_size = orig_streambuf.pubseekoff(0, std::ios::end, std::ios::in);
      orig_streambuf.pubseekoff(28, std::ios::beg, std::ios::in);
      if (file_size == pos_type(off_type(-1))) {
        buffer.resize(0);
        return;
      }
    }

protected:
  int_type l2_decrypt_streambuf::underflow() {
    if (buffer.size() == 0)
      return traits_type::eof();

    // Read up to buffer.size() from the origial streambuf.
    std::streamsize len = orig_streambuf.sgetn(&buffer.front(), buffer.size());
    if (len <= 20)
      return traits_type::eof();

    // Check to see if we read up to the end.
    pos_type loc = orig_streambuf.pubseekoff(0, std::ios::cur, std::ios::in);
    if (loc > (file_size - off_type(20)))
      loc -= (file_size - off_type(20));
    else
      loc = 0;

    // Set our buffer.
    setg(&buffer.front(), &buffer.front(), &buffer.front() + (len - loc));

    // Decrypt the buffer.
    for (auto i = buffer.begin(), e = buffer.end(); i != e; ++i) {
      *i ^= key;
    }

    return traits_type::not_eof(buffer.front());
  }

  pos_type l2_decrypt_streambuf::seekpos(pos_type sp, std::ios::openmode which) {
    // Only change input pos.
    if (~which & std::ios::in)
      return pos_type(off_type(-1));

    // Check bounds.
    if (sp > file_size - off_type(28 + 20))
      return pos_type(off_type(-1));

    // Seek the underlying buffer.
    pos_type pos = orig_streambuf.pubseekpos(sp + off_type(28), which);
    if (pos == pos_type(off_type(-1)))
      return pos;

    // Update the decrypted buffer.
    // TODO: Make this more efficent in the case where sp is within the current
    //       buffer.
    underflow();

    return pos - off_type(28);
  }

  pos_type l2_decrypt_streambuf::seekoff(off_type off, std::ios::seekdir way, std::ios::openmode which) {
    // Only change input pos.
    if (~which & std::ios::in)
      return pos_type(off_type(-1));

    // Seek the underlying buffer.
    pos_type pos = orig_streambuf.pubseekoff(off, way, which);
    if (pos == pos_type(off_type(-1)))
      return pos;

    // Update the decrypted buffer.
    // TODO: Make this more efficent in the case where off is within the current
    //       buffer.
    underflow();

    return pos - off_type(28);
  }

private:
  static const size_t BUFF_SIZE = 4096;
  std::basic_streambuf<char> &orig_streambuf;
  std::vector<char> buffer;
  uint8_t key;
  pos_type file_size;
};

template<class ExtractAsT, class StoreToT>
struct ExtractHelper {
  StoreToT &storeTo;
  ExtractHelper(StoreToT &st) : storeTo(st) {}
};

template<class ExtractAsT, class StoreToT>
std::istream &operator >>(std::istream &is, ExtractHelper<ExtractAsT, StoreToT> &eh) {
  ExtractAsT value;
  is >> value;
  eh.storeTo = value;
  return is;
}

// This exists for template argument deduction.
template<class ExtractAsT, class StoreToT>
ExtractHelper<ExtractAsT, StoreToT> Extract(StoreToT &st) {
  return ExtractHelper<ExtractAsT, StoreToT>(st);
}

template<class ExtractSizeAsT, class ExtractElementAsT, class StoreToT>
struct ExtractArrayHelper {
  StoreToT &storeTo;
  ExtractArrayHelper(StoreToT &st) : storeTo(st) {}
};

template<class ExtractSizeAsT, class ExtractElementAsT, class StoreToT, class IStreamT>
IStreamT &operator >>(IStreamT &is, ExtractArrayHelper< ExtractSizeAsT
                                                      , ExtractElementAsT
                                                      , StoreToT> &eah) {
  ExtractSizeAsT size_val;
  is >> size_val;
  int32_t size = size_val;
  eah.storeTo.reserve(size);
  for (int32_t i = 0; i < size; ++i) {
    ExtractElementAsT element;
    is >> element;
    eah.storeTo.push_back(element);
  }
  return is;
}

template<class ExtractSizeAsT, class ExtractElementAsT, class StoreToT>
ExtractArrayHelper<ExtractSizeAsT, ExtractElementAsT, StoreToT> ExtractArray(StoreToT &st) {
  return ExtractArrayHelper<ExtractSizeAsT, ExtractElementAsT, StoreToT>(st);
}

struct GUID {
  uint32_t A, B, C, D;
  friend std::istream &operator >>(std::istream &is, GUID &guid) {
    is >> Extract<ulittle32_t>(guid.A)
       >> Extract<ulittle32_t>(guid.B)
       >> Extract<ulittle32_t>(guid.C)
       >> Extract<ulittle32_t>(guid.D);
    return is;
  }
};

struct GenerationInfo {
  int32_t export_count;
  int32_t name_count;
  friend std::istream &operator >>(std::istream &is, GenerationInfo &g) {
    is >> Extract<little32_t>(g.export_count)
       >> Extract<little32_t>(g.name_count);
    return is;
  }
};

struct Header {
  int32_t tag;
  int16_t file_version;
  int16_t licensee_mode;
  uint32_t package_flags;
  int32_t name_count;
  int32_t name_offset;
  int32_t export_count;
  int32_t export_offset;
  int32_t import_count;
  int32_t import_offset;
  GUID guid;
  int32_t generation_count;
  std::vector<GenerationInfo> generations;
  friend std::istream &operator >>(std::istream &is, Header &h) {
    is >> Extract<little32_t>(h.tag)
       >> Extract<little16_t>(h.file_version)
       >> Extract<little16_t>(h.licensee_mode)
       >> Extract<ulittle32_t>(h.package_flags)
       >> Extract<little32_t>(h.name_count)
       >> Extract<little32_t>(h.name_offset)
       >> Extract<little32_t>(h.export_count)
       >> Extract<little32_t>(h.export_offset)
       >> Extract<little32_t>(h.import_count)
       >> Extract<little32_t>(h.import_offset)
       >> h.guid
       >> Extract<little32_t>(h.generation_count);
    for (int i = 0; i < h.generation_count; ++i) {
      GenerationInfo gi;
      is >> gi;
      h.generations.push_back(std::move(gi));
    }
    return is;
  }
};

struct Index {
  int32_t value;

  operator int32_t() const {
    return value;
  }

  friend std::istream &operator >>(std::istream &is, Index &i) {
    uint8_t b;
    is >> Extract<ulittle8_t>(b);
    int neg = b & (1 << 7);
    int index = b & 0x3f;

    if (b & (1 << 6)) {
        int shift = 6;
        int data;
        int j = 1;
        do {
            is >> Extract<ulittle8_t>(b);
            data = b & 0x7f;
            data <<= shift;
            index |= data;
            shift += 7;
        } while ((b & (1 << 7)) && (shift < 32));
    }

    if (neg)
        index = -index;

    i.value = index;
    return is;
  }
};

struct NameEntry {
  std::string name;
  uint32_t flags;

  friend std::istream &operator >>(std::istream &is, NameEntry &ne) {
    is >> ExtractArray<Index, char>(ne.name);
    is >> Extract<ulittle32_t>(ne.flags);
    // Remove the trailing 0.
    ne.name.pop_back();
    return is;
  }
};

struct Export {
  Index class_;
  Index super;
  int32_t package;
  Index name;
  uint32_t flags;
  Index size;
  Index offset;

  friend std::istream &operator >>(std::istream &is, Export &e) {
    is >> e.class_
       >> e.super
       >> Extract<little32_t>(e.package)
       >> e.name
       >> Extract<ulittle32_t>(e.flags)
       >> e.size;
    if (e.size > 0)
      is >> e.offset;
    return is;
  }
};

struct Import {
  Index class_package;
  Index class_name;
  int32_t package;
  Index object_name;

  friend std::istream &operator >>(std::istream &is, Import &i) {
    is >> i.class_package
       >> i.class_name
       >> Extract<little32_t>(i.package)
       >> i.object_name;
    return is;
  }
};

struct Archive;
struct Property;

// Map package name to Archive.
extern std::map<std::string, Archive*> packages;
// The file path to the root Lineage II folder (contains system, MAPS, Textures, etc...).
extern std::string L2Path;

// Base class of the unreal object hierarchy.
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

  virtual ~UObject() {}
  virtual void Serialize(Archive &ar);
  // Returns true if consumed.
  virtual bool SetProperty(const Property &p) {
    return false;
  }
  friend Archive &operator >>(Archive &ar, UObject &o) {
    o.Serialize(ar);
    return ar;
  }

  uint32_t flags;
  StringRef name;
  Archive *archive;
};

bool operator <(const Export &a, const Export &b) {
  assert(a.name != 0 && b.name != 0 && "Got invalid name!");
  return a.name < b.name;
}

struct Archive {
  Archive(std::istream &is)
  : is(is) {
    // Read header.
    is >> header;

    // Allocate space.
    names.reserve(header.name_count);
    exports.reserve(header.export_count);
    imports.reserve(header.import_count);

    // Read tables.
    is.seekg(header.name_offset);
    for (int32_t i = 0; i < header.name_count; ++i) {
      NameEntry ne;
      is >> ne;
      names.push_back(ne);
    }
    is.seekg(header.import_offset);
    for (int32_t i = 0; i < header.import_count; ++i) {
      Import im;
      is >> im;
      imports.push_back(im);
    }
    is.seekg(header.export_offset);
    for (int32_t i = 0; i < header.export_count; ++i) {
      Export e;
      is >> e;
      exports.push_back(e);
    }
  }

  StringRef GetName(int index) {
    return names[index].name;
  }

  Index GetClassNameIndex(int class_index) {
    assert(class_index != 0 && "");
    if (class_index < 0)
      return imports[-class_index - 1].object_name;
    else
      return exports[class_index - 1].name;
  }

  StringRef GetClassName(int index) {
    if (index == 0)
      return "None";
    return GetName(GetClassNameIndex(index));
  }

  // Get an object from this archive OR another archive.
  UObject *GetObject(int32_t index) {
    if (index > 0)
      return GetObject(exports[index]);
    else if (index < 0) {
      // It's in another archive. See if it's already loaded. If not, load it.

    }
    return nullptr;
  }

  UObject *GetObject(const Export &e) {
    // Check if it has already been loaded.
    auto i = loaded_objects.find(e);
    if (i != loaded_objects.end())
      return i->second;

    // Load it.
    UObject *obj = ReadExport(e);
    if (obj)
      loaded_objects[e] = obj;
    return obj;
  }

  std::vector<Export>::iterator FindExport(StringRef name) {
    for (auto i = exports.begin(), e = exports.end(); i != e; ++i)
      if (GetName(i->name) == name)
        return i;
    return exports.end();
  }

  template<class T>
  T *LoadExport(StringRef name) {
    auto e = FindExport(name);
    if (e == exports.end())
      return nullptr;
    return dynamic_cast<T*>(GetObject(*e));
  }

  template<class T>
  void LoadExports(StringRef class_name, T &result) {
    // Lookup class name index.
    int32_t class_name_index = -1;
    for (std::vector<NameEntry>::size_type i = 0; i < names.size(); ++i) {
      if (names[i].name == class_name) {
        class_name_index = i;
        break;
      }
    }
    if (class_name_index == -1)
      return;

    // Load all exports with this class name index.
    for (auto i = exports.begin(), e = exports.end(); i != e; ++i) {
      if (GetClassNameIndex(i->class_) == class_name_index) {
        result.push_back(dynamic_cast<T::value_type>(GetObject(*i)));
      }
    }
  }

  std::istream &is;

private:
  UObject *ReadExport(const Export &e);

  Header header;
  std::vector<NameEntry> names;
  std::vector<Export> exports;
  std::vector<Import> imports;
  std::map<Export, UObject*> loaded_objects;
};

Archive &operator >>(Archive &ar, Index &i) {
  ar.is >> i;
  return ar;
}

Archive &operator >>(Archive &ar, StringRef &sr) {
  Index i;
  ar.is >> i;
  sr = ar.GetName(i);
  return ar;
}

struct ArrayIndex {
  int32_t value;

  operator int32_t() const {
    return value;
  }

  friend Archive &operator >>(Archive &ar, ArrayIndex &ai) {
    uint8_t b;
    ar.is >> Extract<ulittle8_t>(b);
    ai.value = b;
    if (b < 128)
      return ar;
    if (b & 0x80) {
      ar.is >> Extract<ulittle8_t>(b);
      ai.value |= uint32_t(b) << 8;
    } else if (b & 0xC0) {
      ar.is >> Extract<ulittle8_t>(b);
      ai.value |= uint32_t(b) << 8;
      ar.is >> Extract<ulittle8_t>(b);
      ai.value |= uint32_t(b) << 16;
      ar.is >> Extract<ulittle8_t>(b);
      ai.value |= uint32_t(b) << 24;
    }
    return ar;
  }
};

struct Rotator {
  int32_t pitch;
  int32_t yaw;
  int32_t roll;

  friend Archive &operator >>(Archive &ar, Rotator &r) {
    ar.is >> Extract<little32_t>(r.pitch)
          >> Extract<little32_t>(r.yaw)
          >> Extract<little32_t>(r.roll);
    return ar;
  }
};

Archive &operator >>(Archive &ar, float &f) {
  ar.is.read(reinterpret_cast<char*>(&f), sizeof(f));
  return ar;
}

struct Vector {
  float X, Y, Z;
  friend Archive &operator >>(Archive &ar, Vector &v) {
    ar >> v.X >> v.Y >> v.Z;
    return ar;
  }

  operator Ogre::Vector3() {
    return Ogre::Vector3(X, Y, Z);
  }
};

template<class T>
struct ObjectRef {
  Archive *archive;
  Index index;
  T *object;

  ObjectRef()
    : archive(nullptr)
    , object(nullptr) {
  }

  T *operator ->() {
    if (object)
      return object;

    // If it's an export, load it from the archive.
    // if (index > 0
  }

  friend Archive &operator >>(Archive &ar, ObjectRef &or) {
    or.archive = &ar;
    ar >> or.index;
    return ar;
  }
};

struct Property {
  StringRef name;
  union {
    uint8_t info;
    struct {
      uint8_t type : 4;
      uint8_t size_type : 3;
      uint8_t is_array : 1;
    };
  };
  StringRef struct_name;
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

  friend Archive &operator >>(Archive &ar, Property &p) {
    ar >> p.name;
    if (p.name == "None")
      return ar;
    ar.is >> Extract<ulittle8_t>(p.info);
    if (p.type == 10)
      ar >> p.struct_name;
    switch (p.size_type) {
      case 0: p.size = 1; break;
      case 1: p.size = 2; break;
      case 2: p.size = 4; break;
      case 3: p.size = 12; break;
      case 4: p.size = 16; break;
      case 5: ar.is >> Extract<ulittle8_t>(p.size); break;
      case 6: ar.is >> Extract<ulittle16_t>(p.size); break;
      case 7: ar.is >> Extract<ulittle32_t>(p.size); break;
    }

    if (p.type == 3) // Boolean.
      return ar;
    if (p.is_array) {
      ar >> p.array_index;
    }

    switch (p.type) {
    case 0x01:
      ar.is >> Extract<ulittle8_t>(p.uint8_t_value);
      break;
    case 0x02:
      ar.is >> Extract<little32_t>(p.int32_t_value);
      break;
    case 0x04:
      ar >> p.float_value;
      break;
    case 0x05:
    case 0x06:
      ar >> p.index_value;
      break;
    case 0x0A:
      if (p.struct_name == "Vector")
        ar >> p.vector_value;
      else if (p.struct_name == "Rotator")
        ar >> p.rotator_value;
      else
        ar.is.seekg(p.size, std::ios::cur);
      break;
    case 0x0C:
      ar >> p.rotator_value;
      break;
    default:
      ar.is.seekg(p.size, std::ios::cur);
    }

    return ar;
  }
};

void UObject::Serialize(Archive &ar) {
  if (flags & RF_HasStack) {
    Index node;
    Index dontcare;
    uint64_t dontcaremoar;
    int32_t stilldontcare;
    ar.is >> node;
    ar.is >> dontcare;
    ar.is >> dontcaremoar;
    ar.is >> stilldontcare;
    if (node != 0)
      ar.is >> dontcare;
  }

  if (!(flags & RF_Native)) {
    Property p;
    do {
      ar >> p;
      SetProperty(p);
    } while(p.name != "None");
  }
}

struct Box {
  Vector min;
  Vector max;
  uint8_t is_valid;

  Box() : is_valid(0) {}

  friend Archive &operator >>(Archive &ar, Box &b) {
    ar >> b.min >> b.max;
    ar.is >> Extract<ulittle8_t>(b.is_valid);
    return ar;
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

  operator Ogre::AxisAlignedBox() {
    return Ogre::AxisAlignedBox(min, max);
  }
};

struct Sphere {
  Vector location;
  float radius;

  friend Archive &operator >>(Archive &ar, Sphere &s) {
    ar >> s.location >> s.radius;
    return ar;
  }
};

struct Plane {
  float X, Y, Z, W;

  friend Archive &operator >>(Archive &ar, Plane &p) {
    ar >> p.X >> p.Y >> p.Z >> p.W;
    return ar;
  }
};

class ULevel : public UObject {
public:

  void Serialize(Archive &ar) {
    UObject::Serialize(ar);
  }
};

class UPrimative : public UObject {
public:
  Box bounding_box;
  Sphere bounding_sphere;

  void Serialize(Archive &ar) {
    UObject::Serialize(ar);
    ar >> bounding_box >> bounding_sphere;
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

  friend Archive &operator >>(Archive &ar, BSPNode &n) {
    ar >> n.plane;
    ar.is >> Extract<ulittle64_t>(n.zone_mask)
          >> Extract<ulittle8_t>(n.node_flags);
    ar >> n.vert_pool
       >> n.surface
       >> n.back
       >> n.front
       >> n.i_plane
       >> n.collision_bound
       >> n.render_bound
       >> n.unknown_point;
    ar.is >> Extract<ulittle32_t>(n.unknown_id)
          >> Extract<ulittle64_t>(n.conn_zones)
          >> Extract<ulittle64_t>(n.vis_zones);
    ar >> n.zone[0]
       >> n.zone[1];
    ar.is >> Extract<ulittle8_t>(n.num_verticies)
          >> Extract<little32_t>(n.leaf[0])
          >> Extract<little32_t>(n.leaf[1]);
    char unk[0xC];
    ar.is.read(unk, sizeof(unk));
    return ar;
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
  Index material;
  uint32_t flags;
  Index base;
  Index normal;
  Index U;
  Index V;
  Index brush_poly;
  Index actor;
  Plane plane;
  uint32_t unk[2];

  friend Archive &operator >>(Archive &ar, BSPSurface &s) {
    ar >> s.material;
    ar.is >> Extract<ulittle32_t>(s.flags);
    ar >> s.base
       >> s.normal
       >> s.U
       >> s.V
       >> s.brush_poly
       >> s.actor
       >> s.plane;
    ar.is >> Extract<ulittle32_t>(s.unk[0])
          >> Extract<ulittle32_t>(s.unk[1]);
    return ar;
  }
};

struct Vertex {
  Index vertex;
  Index side;

  friend Archive &operator >>(Archive &ar, Vertex &v) {
    ar >> v.vertex
       >> v.side;
    return ar;
  }
};

class UModel : public UPrimative {
public:
  std::vector<Vector> vectors;
  std::vector<Vector> points;
  std::vector<BSPNode> nodes;
  std::vector<BSPSurface> surfaces;
  std::vector<Vertex> vertexes;

  void Serialize(Archive &ar) {
    UPrimative::Serialize(ar);

    ar >> ExtractArray<Index, Vector>(vectors)
       >> ExtractArray<Index, Vector>(points)
       >> ExtractArray<Index, BSPNode>(nodes)
       >> ExtractArray<Index, BSPSurface>(surfaces)
       >> ExtractArray<Index, Vertex>(vertexes);
  }

  Box getRegionAABB() const {
    int32_t minX = (regionX - 20) * 32768;
    int32_t minY = (regionY - 18) * 32768;
    Box b;
    b.min.X = minX;
    b.min.Y = minY;
    b.min.Z = -16383;
    b.max.X = minX + 32768;
    b.max.Y = minY + 32768;
    b.max.Z = 16383;
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

UObject *Archive::ReadExport(const Export &e) {
  std::string class_name = GetClassName(e.class_);
  UObject *ret = nullptr;
  is.seekg(std::istream::pos_type(e.offset));
  if (class_name == "Level") {
    ret = new ULevel;
  } else if (class_name == "Model") {
    ret = new UModel;
  }
  if (ret) {
    ret->flags = e.flags;
    ret->name = GetName(e.name);
    ret->archive = this;
    *this >> *ret;
  }
  return ret;
}

}

#endif
