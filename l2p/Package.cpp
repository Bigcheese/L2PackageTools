//===- Package.cpp - Lineage II client package reader -----------*- C++ -*-===//
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

#include "Package.h"
#include "PackedEndian.h"

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

class l2_decrypt_streambuf : public std::streambuf
{
public:
  typedef std::streambuf::traits_type traits_type;
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
}

Package &Package::operator >>(Name &n) {
  Index i;
  *this >> i;
  n = name_map[i];
  return *this;
}

Package &Package::operator >>(Index &i) {
  uint8_t b;
  *input >> Extract<ulittle8_t>(b);
  int neg = b & (1 << 7);
  int index = b & 0x3f;

  if (b & (1 << 6)) {
      int shift = 6;
      int data;
      int j = 1;
      do {
          *input >> Extract<ulittle8_t>(b);
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
          new Package(path.string(), *GNames.insert(name).first));
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

namespace {
struct NameEntry {
  std::string name;
  uint32_t flags;

  friend Package &operator >>(Package &p, NameEntry &ne) {
    p >> ExtractArray<Index, char>(ne.name);
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
  std::unique_ptr<std::basic_filebuf<char>>
    file_buf(new std::basic_filebuf<char>);
  if (file_buf->open(path.str().c_str(), std::ios::in | std::ios::binary) == nullptr)
    throw std::runtime_error("Failed to open file");

  // Setup the decryption streambuf.
  input_chain.emplace_back(new l2_decrypt_streambuf(*file_buf));
  input_chain.emplace_back(file_buf.release());

  // Setup the actual input stream.
  input.reset(new std::istream(input_chain.front().get()));

  // Read the package metadata.
  *this >> header;
  input->seekg(header.name_offset);
  for (int32_t i = 0; i < header.name_count; ++i) {
    NameEntry ne;
    *this >> ne;
    name_map.push_back(*GNames.insert(std::move(ne.name)).first);
  }
  input->seekg(header.import_offset);
  for (int32_t i = 0; i < header.import_count; ++i) {
    Import im;
    *this >> im;
    import_table.push_back(std::move(im));
  }
  input->seekg(header.export_offset);
  for (int32_t i = 0; i < header.export_count; ++i) {
    Export e;
    *this >> e;
    export_table.push_back(std::move(e));
  }
}
