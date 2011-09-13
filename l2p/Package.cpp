//===- Package.cpp - Lineage II client package reader -----------*- C++ -*-===//
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

#include "l2p/Package.h"
#include "l2p/UObject.h"

#include "boost/filesystem.hpp"

#include <array>
#include <exception>
#include <fstream>
#include <sstream>

using namespace l2p;

std::string Package::GL2RootDir;
std::unordered_map<std::string, std::shared_ptr<Package>> Package::GPackages;
std::unordered_set<std::string> Package::GNames;

namespace {
struct L2Paths {
  const char *dir;
  const char *ext;

  L2Paths(const char *d, const char *e)
    : dir(d)
    , ext(e) {}
};

std::array<L2Paths, 6> dirs = {
  L2Paths("Animations", "ukx"),
  L2Paths("MAPS", "unr"),
  L2Paths("Sounds", "uax"),
  L2Paths("StaticMeshes", "usx"),
  L2Paths("SysTextures", "utx"),
  L2Paths("Textures", "utx")
};

class l2_decrypt_filter : public io::multichar_filter<io::input_seekable> {
public:
  l2_decrypt_filter()
    : initialized(false) {}

  template<class SourceT>
  std::streamsize read(SourceT &d, char *buffer, std::streamsize size) {
    if (!initialized) {
      io::seek(d, 0, std::ios::beg, std::ios::in);
      std::array<char, 22> target_header = {
        0x4C, 0x00, 0x69, 0x00, 0x6E, 0x00, 0x65, 0x00,
        0x61, 0x00, 0x67, 0x00, 0x65, 0x00, 0x32, 0x00,
        0x56, 0x00, 0x65, 0x00, 0x72, 0x00
      }; // Lineage2Ver
      // Read the version from orig_streambuf.
      std::array<char, 22> header_buf;
      std::array<char, 6>  version_buf;
      io::read(d, header_buf.data(), 22);
      io::read(d, version_buf.data(), 6);
      if (header_buf != target_header)
        return -1;

      // Get the version as an integer.
      std::string ver;
      ver += version_buf[0];
      ver += version_buf[2];
      ver += version_buf[4];

      std::istringstream out(ver);
      int archive_version;
      out >> archive_version;
      if (!out)
        return -1;

      // Get key from version.
      switch (archive_version) {
      case 111: key = 0xAC; break;
      case 121: {
          uint8_t val = io::get(d);
          key = val ^ 0xC1;
          break;
        }
      default:
        return -1;
      }

      wide_key = key | key << 8 | key << 16 | key << 24;

      // Get the total file size so we know when to return EOF.
      file_size = io::seek(d, 0, std::ios::end, std::ios::in);
      io::seek(d, 28, std::ios::beg, std::ios::in);
      if (file_size == -1)
        return -1;

      initialized = true;
    }

    std::streamsize len = io::read(d, buffer, size);
    std::streamsize left = len;
    if (len != -1) {
      for (std::streamsize i = 0, e = std::min<std::streamsize>(uintptr_t(buffer) & sizeof(uint32_t), left); i != e; ++i) {
        *buffer++ ^= key;
        --left;
      }
      for (std::streamsize i = 0, e = left / sizeof(uint32_t); i != e; ++i) {
        *reinterpret_cast<uint32_t*>(buffer) ^= wide_key;
        buffer += sizeof(uint32_t);
        left -= sizeof(uint32_t);
      }
      for (std::streamsize i = 0, e = left; i != e; ++i) {
        *buffer++ ^= key;
      }
    }
    return len;
  }

  template<class SourceT>
  std::streampos seek(SourceT &d, io::stream_offset off, std::ios::seekdir way) {
    if (way == std::ios::beg)
      return io::seek(d, off + 28, way, std::ios::in) - io::stream_offset(28);
    if (way == std::ios::cur)
      return io::seek(d, off, way, std::ios::in) - io::stream_offset(28);
    if (way == std::ios::end)
      return io::seek(d, off - 20, way, std::ios::in) - io::stream_offset(28);
    return -1;
  }

private:
  bool initialized;
  uint8_t key;
  uint32_t wide_key;
  std::streamsize file_size;
};
}

Package &Package::operator >>(Name &n) {
  Index i;
  *this >> i;
  n = name_map[i];
  return *this;
}

Package &Package::operator >>(Index &i) {
  uint8_t b;
  input >> Extract<ulittle8_t>(b);
  int neg = b & (1 << 7);
  int index = b & 0x3f;

  if (b & (1 << 6)) {
      int shift = 6;
      int data;
      int j = 1;
      do {
          input >> Extract<ulittle8_t>(b);
          data = b & 0x7f;
          data <<= shift;
          index |= data;
          shift += 7;
      } while ((b & (1 << 7)) && (shift < 32));
  }

  if (neg)
      index = -index;

  i.value = index;
  return *this;
}

Package &Package::operator >>(Import &i) {
  *this >> i.class_package
        >> i.class_name
        >> Extract<little32_t>(i.package)
        >> i.object_name;
  return *this;
}

Package &Package::operator >>(Export &e) {
  *this >> e.class_
        >> e.super
        >> Extract<little32_t>(e.package)
        >> e.name
        >> Extract<ulittle32_t>(e.flags)
        >> e.size;
  if (e.size > 0)
    *this >> e.offset;
  return *this;
}

Package &Package::operator >>(GUID &g) {
  *this >> Extract<ulittle32_t>(g.A)
        >> Extract<ulittle32_t>(g.B)
        >> Extract<ulittle32_t>(g.C)
        >> Extract<ulittle32_t>(g.D);
  return *this;
}

Package &Package::operator >>(GenerationInfo &g) {
  *this >> Extract<little32_t>(g.export_count)
        >> Extract<little32_t>(g.name_count);
  return *this;
}

Package &Package::operator >>(Header &h) {
  *this >> Extract<little32_t>(h.tag)
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
    *this >> gi;
    h.generations.push_back(std::move(gi));
  }
  return *this;
}

bool Package::Initialize(StringRef root_path) {
  // Verify that all dirs are under this root.
  for (auto dir = dirs.begin(), e = dirs.end(); dir != e; ++dir) {
    boost::filesystem::path p(root_path.str());
    p /= dir->dir;
    boost::system::error_code ec;
    if (!boost::filesystem::exists(p, ec))
      return false;
  }

  GL2RootDir = root_path;

  return true;
}

Package *Package::GetPackage(StringRef name) {
  if (GL2RootDir.empty())
    return nullptr;

  // See if it has already been opened.
  auto p = GPackages.find(name);
  if (p != GPackages.end())
    return p->second.get();

  // Find the package under dirs.
  for (auto dir = dirs.begin(), e = dirs.end(); dir != e; ++dir) {
    boost::filesystem::path path(GL2RootDir);
    path /= dir->dir;
    path /= name.str() + '.' + dir->ext;
    boost::system::error_code ec;
    if (boost::filesystem::exists(path, ec)) {
      // Load the package.
      try {
        std::shared_ptr<Package> p(
          new Package(path.string(), Name::make(*GNames.insert(name).first)));
        GPackages.insert(std::make_pair(name, p));
        return p.get();
      } catch (const std::runtime_error &) {
        return nullptr;
      }
    }
  }

  // Package not found.
  return nullptr;
}

Name Package::GetName(StringRef name) {
  return Name::make(*GNames.insert(name).first);
}

namespace {
struct NameEntry {
  std::string name;
  uint32_t flags;

  friend Package &operator >>(Package &p, NameEntry &ne) {
    p >> ExtractArray<Index, little8_t>(ne.name);
    p >> Extract<ulittle32_t>(ne.flags);
    // Remove the trailing 0.
    ne.name.pop_back();
    return p;
  }
};
}

Package::Package(StringRef path, Name name)
  : name(name) {
  // Get file streambuf.
  input.push(l2_decrypt_filter());
  input.push(io::file_source(path, std::ios::in | std::ios::binary));

  // Read the package metadata.
  *this >> header;
  input.seekg(header.name_offset);
  for (int32_t i = 0; i < header.name_count; ++i) {
    NameEntry ne;
    *this >> ne;
    name_map.push_back(Name::make(*GNames.insert(std::move(ne.name)).first));
  }
  input.seekg(header.import_offset);
  for (int32_t i = 0; i < header.import_count; ++i) {
    Import im;
    *this >> im;
    import_table.push_back(std::move(im));
  }
  input.seekg(header.export_offset);
  for (int32_t i = 0; i < header.export_count; ++i) {
    Export e;
    *this >> e;
    export_table.push_back(std::move(e));
  }
}

std::shared_ptr<UObject> Package::GetObject(int index) {
  assert(index != 0 && "Index must be non-zero!");

  if (index < 0) {
    index = -index - 1;
    if (std::size_t(index) >= import_table.size())
      throw std::runtime_error("Invalid index");
    Import &import = import_table[index];
    if (import.package == 0)
      throw std::runtime_error("Invalid package?");

    // Find the name of the package this import is in.
    Import *package_import = &import;
    do {
      if (std::size_t(-package_import->package) > import_table.size())
        throw std::runtime_error("Invalid package?");
      package_import = &import_table[-package_import->package - 1];
    } while (package_import->package != 0);
    // package_import is now the import entry for the physical package.
    Package *target_package = Package::GetPackage(package_import->object_name);
    if (!target_package)
      return nullptr;
    return target_package->GetObject(import.object_name);
  }

  if (std::size_t(index) > export_table.size())
    throw std::runtime_error("Invalid index");
  return DeserializeExport(export_table[index - 1]);
}

std::shared_ptr<UObject> Package::GetObject(Name name) {
  for (auto i = export_table.begin(), e = export_table.end(); i != e; ++i) {
    if (i->name == name && GetObjectName(i->class_) != "Package")
      return DeserializeExport(*i);
  }
  return nullptr;
}

std::shared_ptr<UObject> Package::DeserializeExport(Export &e) {
  if (e.object)
    return e.object;

  Name class_name = GetObjectName(e.class_);

  std::shared_ptr<UObject> ret(nullptr);
  if (class_name == "Model") {
    ret.reset(new UModel);
  } else if (class_name == "TerrainInfo") {
    ret.reset(new ATerrainInfo);
  } else if (class_name == "Texture") {
    ret.reset(new UTexture);
  } else if (class_name == "StaticMesh") {
    ret.reset(new UStaticMesh);
  } else if (  class_name == "StaticMeshActor"
            || class_name == "MovableStaticMeshActor") {
    ret.reset(new AStaticMeshActor);
  } else if (class_name == "Brush") {
    ret.reset(new ABrush);
  } else if (class_name == "BlockingVolume") {
    ret.reset(new ABlockingVolume);
  } else if (class_name == "WaterVolume") {
    ret.reset(new AWaterVolume);
  } else if (class_name == "Shader") {
    ret.reset(new UShader);
  } else if (class_name == "FinalBlend") {
    ret.reset(new UFinalBlend);
  } else {
    // Unknown, but it must be a child of UObject, so use that.
    ret.reset(new UObject);
  }
  if (ret) {
    ret->flags = e.flags;
    ret->name = e.name;
    ret->package = this;
    if (e.size > 0)
      input.seekg(std::istream::off_type(e.offset));
    *this >> *ret;
  }
  e.object = ret;
  return e.object;
}
