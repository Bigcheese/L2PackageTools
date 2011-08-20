/*
-----------------------------------------------------------------------------
Filename:    TutorialApplication.cpp
-----------------------------------------------------------------------------

This source file is part of the
   ___                 __    __ _ _    _
  /___\__ _ _ __ ___  / / /\ \ (_) | _(_)
 //  // _` | '__/ _ \ \ \/  \/ / | |/ / |
/ \_// (_| | | |  __/  \  /\  /| |   <| |
\___/ \__, |_|  \___|   \/  \/ |_|_|\_\_|
      |___/
      Tutorial Framework
      http://www.ogre3d.org/tikiwiki/
-----------------------------------------------------------------------------
*/
#include "TutorialApplication.h"
#include "PackedEndian.h"

#include <array>
#include <cstdint>
#include <exception>
#include <fstream>
#include <streambuf>
#include <vector>

namespace L2Package {
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
    , buffer(BUFF_SIZE)
    , pack(nullptr) {
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

template<class ExtractSizeAsT, class ExtractElementAsT, class StoreToT>
std::istream &operator >>(std::istream &is, ExtractArrayHelper< ExtractSizeAsT
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

struct Package {
  Header header;
  std::vector<NameEntry> names;
  std::vector<Export> exports;
  std::vector<Import> imports;
};

std::istream &operator >>(std::istream &is, Package &p) {
  is >> p.header;
  is.seekg(p.header.name_offset);
  for (int32_t i = 0; i < p.header.name_count; ++i) {
    NameEntry ne;
    is >> ne;
    p.names.push_back(ne);
  }
  is.seekg(p.header.export_offset);
  for (int32_t i = 0; i < p.header.export_count; ++i) {
    Export e;
    is >> e;
    p.exports.push_back(e);
  }
  is.seekg(p.header.import_offset);
  for (int32_t i = 0; i < p.header.import_count; ++i) {
    Import im;
    is >> im;
    p.imports.push_back(im);
  }
  return is;
}

}

//-------------------------------------------------------------------------------------
TutorialApplication::TutorialApplication(void)
{
}
//-------------------------------------------------------------------------------------
TutorialApplication::~TutorialApplication(void)
{
}

//-------------------------------------------------------------------------------------
void TutorialApplication::createScene(void)
{
  /*Ogre::MeshPtr mesh = Ogre::MeshManager::getSingleton().createManual("blah", "General");
  Ogre::SubMesh *subMesh = mesh->createSubMesh();*/

  std::fstream file("G:\\Program Files (x86)\\NCsoft\\Lineage II\\MAPS\\11_23.unr", std::ios::binary | std::ios::in);
  L2Package::l2_decrypt_streambuf l2ds(*file.rdbuf());
  std::istream package_stream(&l2ds);

  L2Package::Package p;
  package_stream >> p;

  Ogre::Entity *ogreHead = mSceneMgr->createEntity("Head", "ogrehead.mesh");
  Ogre::SceneNode *headNode = mSceneMgr->getRootSceneNode()->createChildSceneNode();
  headNode->attachObject(ogreHead);
  mSceneMgr->setAmbientLight(Ogre::ColourValue(0.5, 0.5, 0.5));
  Ogre::Light *l = mSceneMgr->createLight("MainLight");
  l->setPosition(20, 80, 50);
}



#if OGRE_PLATFORM == OGRE_PLATFORM_WIN32
#define WIN32_LEAN_AND_MEAN
#include "windows.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif

#if OGRE_PLATFORM == OGRE_PLATFORM_WIN32
    INT WINAPI WinMain( HINSTANCE hInst, HINSTANCE, LPSTR strCmdLine, INT )
#else
    int main(int argc, char *argv[])
#endif
    {
        // Create application object
        TutorialApplication app;

        try {
            app.go();
        } catch( Ogre::Exception& e ) {
#if OGRE_PLATFORM == OGRE_PLATFORM_WIN32
            MessageBox( NULL, e.getFullDescription().c_str(), "An exception has occured!", MB_OK | MB_ICONERROR | MB_TASKMODAL);
#else
            std::cerr << "An exception has occured: " <<
                e.getFullDescription().c_str() << std::endl;
#endif
        }

        return 0;
    }

#ifdef __cplusplus
}
#endif
