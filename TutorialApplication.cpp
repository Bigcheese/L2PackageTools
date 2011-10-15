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
#include "DynamicLines.h"
#undef GetObject
#include "l2p/UObject.h"

#include "boost/filesystem.hpp"
#include "boost/program_options.hpp"

#include <sstream>
#include <vector>

namespace po = boost::program_options;
namespace fs = boost::filesystem;

Ogre::Vector3 ogre_cast(const l2p::Vector &v) {
  return Ogre::Vector3(v.X, v.Y, v.Z);
}

Ogre::AxisAlignedBox ogre_cast(const l2p::Box &b) {
  return Ogre::AxisAlignedBox(ogre_cast(b.min), ogre_cast(b.max));
}

Ogre::TexturePtr loadTexture(std::shared_ptr<l2p::UTexture> texture) {
  if (texture && texture->mips.size() > 0) {
    Ogre::PixelFormat format = Ogre::PF_UNKNOWN;
    switch (texture->Format) {
      case l2p::UTexture::TEXF_DXT1:
        format = Ogre::PF_DXT1;
        break;
      case l2p::UTexture::TEXF_DXT3:
        format = Ogre::PF_DXT3;
        break;
      case l2p::UTexture::TEXF_DXT5:
        format = Ogre::PF_DXT5;
        break;
      case l2p::UTexture::TEXF_RGBA8:
        format = Ogre::PF_R8G8B8A8;
        break;
      default:
        std::stringstream ss;
        ss << texture->GetPath() << ": " << "Unknown texture format " << texture->Format;
        Ogre::LogManager::getSingleton().logMessage(Ogre::LML_NORMAL, ss.str());
        break;
    }
    if (format != Ogre::PF_UNKNOWN) {
      Ogre::TexturePtr tptr = Ogre::TextureManager::getSingleton().getByName(texture->GetPath());
      if (tptr.isNull()) {
        std::vector<uint8_t> data;
        for (auto i = texture->mips.begin(), e = texture->mips.end(); i != e; ++i) {
          data.insert(data.end(), i->data.begin(), i->data.end());
        }
        Ogre::DataStreamPtr pmds(new Ogre::MemoryDataStream(&data.front(), data.size()));
        Ogre::Image img;
        img.loadRawData(pmds, texture->USize, texture->VSize, 1, format, 1, texture->mips.size() - 1);
        tptr = Ogre::TextureManager::getSingleton().loadImage(texture->GetPath(), Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, img);
      }
      return tptr;
    }
  }
  return Ogre::TexturePtr(nullptr);
}

//-------------------------------------------------------------------------------------
TutorialApplication::TutorialApplication(void)
{
}
//-------------------------------------------------------------------------------------
TutorialApplication::~TutorialApplication(void)
{
}

bool ignoreNode(l2p::UModel *m, l2p::BSPNode &n, l2p::BSPSurface &s) {
  if (s.flags & l2p::PF_WeDontCareAbout || !s.material.index)
    return true;
  for (int i = 0; i < n.num_verticies; ++i) {
    Ogre::Vector3 loc = ogre_cast(m->points[m->vertexes[n.vert_pool + i].vertex]);
    Ogre::AxisAlignedBox box = ogre_cast(m->getRegionAABB());
    if (!box.contains(loc))
      return true;
  }
  return false;
}

void assignActorPropsToNode(std::shared_ptr<l2p::AActor> a, Ogre::SceneNode *n) {
  n->setPosition(ogre_cast(a->location) - ogre_cast(a->pre_pivot));
  n->roll(Ogre::Radian(-0.000096f) * a->rotation.yaw);
  n->yaw(Ogre::Radian(0.000096f) * a->rotation.pitch);
  n->pitch(Ogre::Radian(-0.000096f) * a->rotation.roll);
  n->setScale(ogre_cast(a->draw_scale_3d) * a->draw_scale);
}

void TutorialApplication::loadMap(l2p::StringRef name) {
  l2p::Package *package = l2p::Package::GetPackage(name);
  if (!package)
    return;

  std::vector<std::shared_ptr<l2p::UModel>> models;
  package->GetObjects("Model", models);

  // Find the largest model. The others aren't right or something like that...
  std::shared_ptr<l2p::UModel> m = nullptr;
  for (auto i = models.begin(), e = models.end(); i != e; ++i) {
    if (!m) {
      m = *i;
      continue;
    }
    if ((*i)->nodes.size() > m->nodes.size())
      m = *i;
  }

  // Get region from name.
  {
    name.substr(0, 2).getAsInteger(10, m->regionX);
    name.substr(3, 2).getAsInteger(10, m->regionY);
    mLoadedRegions.push_back(std::make_pair(m->regionX, m->regionY));
  }

  if (m->points.size() != 0)
    loadBSP(m);

  std::vector<std::shared_ptr<l2p::AVolume>> volumes;
  package->GetObjects("BlockingVolume", volumes);
  package->GetObjects("WaterVolume", volumes);
  for (auto i = volumes.begin(), e = volumes.end(); i != e; ++i) {
    std::shared_ptr<l2p::UModel> m = (*i)->brush;
    if (m) {
      Ogre::SceneNode *n = loadBSP(m, false);
      if (n) {
        assignActorPropsToNode(*i, n);
        if (std::dynamic_pointer_cast<l2p::AWaterVolume>(*i))
          dynamic_cast<Ogre::Entity*>(
            n->getAttachedObject(0))->setMaterialName("Volume/Water");
        else
          dynamic_cast<Ogre::Entity*>(
            n->getAttachedObject(0))->setMaterialName("Volume/Blocking");
      }
    }
  }

  std::vector<std::shared_ptr<l2p::ATerrainInfo>> terrains;
  package->GetObjects("TerrainInfo", terrains);

  for (auto i = terrains.begin(), e = terrains.end(); i != e; ++i) {
    loadTerrain(*i);
  }

  std::vector<std::shared_ptr<l2p::AStaticMeshActor>> smeshes;
  package->GetObjects("StaticMeshActor", smeshes);
  package->GetObjects("MovableStaticMeshActor", smeshes);

  for (auto i = smeshes.begin(), e = smeshes.end(); i != e; ++i) {
    if (  (*i)->bHidden
       || (*i)->bDeleteMe
       || mIgnoreNonCollidable
        && (!(*i)->bCollideActors
         || !(*i)->bBlockActors
         || !(*i)->bBlockPlayers)
       )
      continue;
    loadStaticMeshActor(*i);
  }
}

Ogre::Real operator |(const Ogre::Vector3 &a, const Ogre::Vector3 &b) {
  return (a.x * b.x) + (a.y * b.y) + (a.z * b.z);
}

struct Surface {
  l2p::BSPSurface *surface;
  std::vector<uint16_t> indicies;
  int index_start;
};

Ogre::SceneNode *TutorialApplication::loadBSP(std::shared_ptr<l2p::UModel> m, bool ignoreNonVisible) {
  l2p::Name name = m->package->name;
  std::vector<float> vertex_data;
  std::vector<uint16_t> index_buf;
  std::vector<Surface> surfaces(m->surfaces.size());
  l2p::Box bounds;

  // Build vertex and index buffer.
  for (auto ni = m->nodes.begin(), ne = m->nodes.end(); ni != ne; ++ni) {
    l2p::BSPNode &n = *ni;
    Surface &s = surfaces[n.surface];
    s.surface = &m->surfaces[n.surface];

    if (ignoreNonVisible && ignoreNode(m.get(), n, *s.surface))
      continue;

    uint32_t vert_start = vertex_data.size() / 8;

    const Ogre::Vector3 uvec = ogre_cast(m->vectors[s.surface->U]);
    const Ogre::Vector3 vvec = ogre_cast(m->vectors[s.surface->V]);
    const Ogre::Vector3 base = ogre_cast(m->points[s.surface->base]);
    int usize = 0;
    int vsize = 0;
    std::shared_ptr<l2p::UMaterial> mat = s.surface->material;
    std::shared_ptr<l2p::UTexture> tex = std::dynamic_pointer_cast<l2p::UTexture>(mat);
    if (tex) {
      usize = tex->USize;
      vsize = tex->VSize;
    }

    if (usize == 0 || vsize == 0)
      usize = vsize = 64;

    // Vertex buffer.
    if (n.num_verticies > 0) {
      l2p::Vector Normal = m->vectors[s.surface->normal];

      for (uint32_t vert_index = 0; vert_index < n.num_verticies; ++vert_index) {
        const l2p::Vector &pos = m->points[m->vertexes[n.vert_pool + vert_index].vertex];
        const Ogre::Vector3 dist(ogre_cast(pos) - base);
        const Ogre::Vector2 tcoord((dist | uvec) / float(usize), (dist | vvec) / float(vsize));
        bounds += pos;
        vertex_data.push_back(pos.X);
        vertex_data.push_back(pos.Y);
        vertex_data.push_back(pos.Z);
        vertex_data.push_back(Normal.X);
        vertex_data.push_back(Normal.Y);
        vertex_data.push_back(Normal.Z);
        vertex_data.push_back(tcoord.x);
        vertex_data.push_back(tcoord.y);
      }

      if (s.surface->flags & l2p::PF_TwoSided) {
        for (uint32_t vert_index = 0; vert_index < n.num_verticies; ++vert_index) {
          const l2p::Vector &pos = m->points[m->vertexes[n.vert_pool + vert_index].vertex];
          const Ogre::Vector3 dist(ogre_cast(pos) - base);
          const Ogre::Vector2 tcoord((dist | uvec) / float(usize), (dist | vvec) / float(vsize));
          vertex_data.push_back(pos.X);
          vertex_data.push_back(pos.Y);
          vertex_data.push_back(pos.Z);
          vertex_data.push_back(Normal.X);
          vertex_data.push_back(Normal.Y);
          vertex_data.push_back(-Normal.Z);
          vertex_data.push_back(tcoord.x);
          vertex_data.push_back(tcoord.y);
        }
      }
    }

    // Index buffer.
    for (int verti = 2; verti < n.num_verticies; ++verti) {
      s.indicies.push_back(vert_start);
      s.indicies.push_back(vert_start + verti - 1);
      s.indicies.push_back(vert_start + verti);
    }
    if (s.surface->flags & l2p::PF_TwoSided) {
      for (int verti = 2; verti < n.num_verticies; ++verti) {
        s.indicies.push_back(vert_start);
        s.indicies.push_back(vert_start + verti);
        s.indicies.push_back(vert_start + verti - 1);
      }
    }
  }

  // Build index buffer from surface indicies;
  uint16_t index_start = 0;
  for (auto i = surfaces.begin(), e = surfaces.end(); i != e; ++i) {
    i->index_start = index_start;
    index_buf.insert(index_buf.end(), i->indicies.begin(), i->indicies.end());
    index_start += i->indicies.size();
  }

  if (vertex_data.size() == 0 || index_buf.size() == 0)
    return nullptr;

  Ogre::MeshPtr mesh = Ogre::MeshManager::getSingleton().createManual(Ogre::String(name) + Ogre::String(m->name), "General");
  Ogre::VertexData  *data = new Ogre::VertexData();
  mesh->sharedVertexData = data;
  data->vertexCount = vertex_data.size() / 8;

  Ogre::VertexDeclaration *decl = data->vertexDeclaration;
  uint32_t offset = 0;
  decl->addElement(0, offset, Ogre::VET_FLOAT3, Ogre::VES_POSITION);
  offset += Ogre::VertexElement::getTypeSize(Ogre::VET_FLOAT3);
  decl->addElement(0, offset, Ogre::VET_FLOAT3, Ogre::VES_NORMAL);
  offset += Ogre::VertexElement::getTypeSize(Ogre::VET_FLOAT3);
  decl->addElement(0, offset, Ogre::VET_FLOAT2, Ogre::VES_TEXTURE_COORDINATES);
  offset += Ogre::VertexElement::getTypeSize(Ogre::VET_FLOAT2);

  Ogre::HardwareVertexBufferSharedPtr vbuf =
    Ogre::HardwareBufferManager::getSingleton().createVertexBuffer(
      offset,
      data->vertexCount,
      Ogre::HardwareBuffer::HBU_STATIC_WRITE_ONLY);
  vbuf->writeData(0, vbuf->getSizeInBytes(), &vertex_data.front(), true);
  data->vertexBufferBinding->setBinding(0, vbuf);

  // Setup index buffer.
  Ogre::HardwareIndexBufferSharedPtr ibuf =
    Ogre::HardwareBufferManager::getSingleton().createIndexBuffer(
      Ogre::HardwareIndexBuffer::IT_16BIT,
      index_buf.size(),
      Ogre::HardwareBuffer::HBU_STATIC_WRITE_ONLY);

  ibuf->writeData(0, ibuf->getSizeInBytes(), &index_buf.front(), true);
  for (auto i = surfaces.begin(), e = surfaces.end(); i != e; ++i) {
    if (i->indicies.size() < 3)
      continue;
    Ogre::SubMesh *subMesh = mesh->createSubMesh();
    subMesh->useSharedVertices = true;
    subMesh->indexData->indexBuffer = ibuf;
    subMesh->indexData->indexStart  = i->index_start;
    subMesh->indexData->indexCount  = i->indicies.size();

    std::shared_ptr<l2p::UMaterial> mat = i->surface->material;
    if (mat) {
      subMesh->setMaterialName(loadMaterial(mat)->getName());
    }
  }

  mesh->_setBounds(Ogre::AxisAlignedBox(bounds.min.X, bounds.min.Y, bounds.min.Z, bounds.max.X, bounds.max.Y, bounds.max.Z));
  mesh->_setBoundingSphereRadius((std::max(bounds.max.X - bounds.min.X, std::max(bounds.max.Y - bounds.min.Y, bounds.max.Z - bounds.min.Z))) / 2.0);

  mesh->load();

  Ogre::Entity *ent = mSceneMgr->createEntity(Ogre::String(name) + Ogre::String(m->name) + "E", Ogre::String(name) + Ogre::String(m->name));
  ent->setUserAny(Ogre::Any(static_cast<l2p::UObject*>(m.get())));
  Ogre::SceneNode *node = mUnrealCordNode->createChildSceneNode();
  node->attachObject(ent);

  return node;
}

struct BufferVert {
  Ogre::Vector3 position;
  Ogre::Vector3 normal;
  Ogre::Vector2 texcoord;
};

void TutorialApplication::loadTerrain(std::shared_ptr<l2p::ATerrainInfo> ti) {
  l2p::Name name = ti->package->name;
  std::vector<BufferVert> vertex_buffer;
  std::vector<uint32_t> index_buffer;
  if (ti->terrain_map->mips.size() == 0)
    return;
  l2p::ulittle16_t *height_map = ti->terrain_map->mips[0].getAs<l2p::ulittle16_t>();
  Ogre::Vector3 location = ogre_cast(ti->location);
  Ogre::Vector3 scale = ogre_cast(ti->terrain_scale);
  int32_t USize = ti->terrain_map->USize;
  int32_t VSize = ti->terrain_map->VSize;

  Ogre::Vector3 translation( location.x - ((USize >> 1) * scale.x)
                           , location.y - ((VSize >> 1) * scale.y)
                           , location.z - (128 * scale.z)
                           );

  float max_hight = (65535.f * (scale.z / 256.f)) + translation.z;
  float min_hight = (0.f * (scale.z / 256.f)) + translation.z;

  vertex_buffer.reserve(USize * VSize);
  Ogre::AxisAlignedBox box;
  Ogre::RenderSystem *rs = Ogre::Root::getSingleton().getRenderSystem();
  for (int y = 0; y < VSize; ++y) {
    for (int x = 0; x < USize; ++x) {
      Ogre::Vector3 v( (x * scale.x) + translation.x
                     , (y * scale.y) + translation.y
                     , (height_map[(y * USize) + x] * (scale.z / 256.f)) + translation.z
                     );
      box.merge(v);
      BufferVert bv = {v, Ogre::Vector3(0.f, 0.f, 0.f), Ogre::Vector2(float(x) / (USize), float(y) / (VSize))};
      vertex_buffer.push_back(bv);
    }
  }

  for (int y = 0; y < VSize - 1; ++y) {
    for (int x = 0; x < USize - 1; ++x) {
      if (!ti->quad_visibility_bitmap[x + (y * USize)])
        continue;
      if (!ti->edge_turn_bitmap[x + (y * USize)]) {
        // First part of quad.
        index_buffer.push_back(x + (y * USize));
        index_buffer.push_back((x + 1) + (y * USize));
        index_buffer.push_back((x + 1) + ((y + 1) * USize));
        // Second part of quad.
        index_buffer.push_back(x + (y * USize));
        index_buffer.push_back((x + 1) + ((y + 1) * USize));
        index_buffer.push_back(x + ((y + 1) * USize));
      } else {
        // First part of quad.
        index_buffer.push_back(x + ((y + 1) * USize));
        index_buffer.push_back(x + (y * USize));
        index_buffer.push_back((x + 1) + (y * USize));
        // Second part of quad.
        index_buffer.push_back(x + ((y + 1) * USize));
        index_buffer.push_back((x + 1) + (y * USize));
        index_buffer.push_back((x + 1) + ((y + 1) * USize));
      }
    }
  }

  // We also have to load the nearby highmaps to handle the edges.
  // TODO: support unloading this data after terrain has been generated.
  // TODO: Make this code not so repeditive.
  l2p::Package *terrain_east = nullptr;
  l2p::Package *terrain_south = nullptr;
  l2p::Package *terrain_south_east = nullptr;
  {
    std::stringstream pname;
    pname << "T_" << ti->map_x + 1 << "_" << ti->map_y;
    terrain_east = l2p::Package::GetPackage(pname.str());
    pname.str(std::string());
    pname << "T_" << ti->map_x << "_" << ti->map_y + 1;
    terrain_south = l2p::Package::GetPackage(pname.str());
    if (terrain_east && terrain_south) {
      pname.str(std::string());
      pname << "T_" << ti->map_x + 1 << "_" << ti->map_y + 1;
      terrain_south_east = l2p::Package::GetPackage(pname.str());
    }
    // Inject the extra verticies.
    if (terrain_south) {
      pname.str(std::string());
      pname << ti->map_x << "_" << ti->map_y + 1;
      std::shared_ptr<l2p::UTexture> tm = std::dynamic_pointer_cast<l2p::UTexture>(terrain_south->GetObject(l2p::Package::GetName(pname.str())));
      if (tm && tm->mips.size() > 0) {
        int y = VSize;
        for (int x = 0; x < USize; ++x) {
          Ogre::Vector3 v( (x * scale.x) + translation.x
                         , (y * scale.y) + translation.y
                         , (tm->mips[0].getAs<l2p::ulittle16_t>()[x] * (scale.z / 256.f)) + translation.z
                         );
          box.merge(v);
          BufferVert bv = {v, Ogre::Vector3(0.f, 0.f, 0.f), Ogre::Vector2(float(x) / (USize), float(y) / (VSize))};
          vertex_buffer.push_back(bv);

          // First part of quad.
          if (x != USize - 1) {
            index_buffer.push_back(x + ((y - 1) * USize));
            index_buffer.push_back((x + 1) + ((y - 1) * USize));
            index_buffer.push_back(vertex_buffer.size() - 1);
          }
          // Second part of quad.
          if (x != 0) {
            index_buffer.push_back(x + ((y - 1) * USize));
            index_buffer.push_back(vertex_buffer.size() - 1);
            index_buffer.push_back(vertex_buffer.size() - 2);
          }
        }
      }
    }

    if (terrain_east) {
      pname.str(std::string());
      pname << ti->map_x + 1 << "_" << ti->map_y;
      std::shared_ptr<l2p::UTexture> tm = std::dynamic_pointer_cast<l2p::UTexture>(terrain_east->GetObject(l2p::Package::GetName(pname.str())));
      if (tm && tm->mips.size() > 0) {
        const int x = USize;
        for (int y = 0; y < VSize; ++y) {
          Ogre::Vector3 v( (x * scale.x) + translation.x
                         , (y * scale.y) + translation.y
                         , (tm->mips[0].getAs<l2p::ulittle16_t>()[y * USize] * (scale.z / 256.f)) + translation.z
                         );
          box.merge(v);
          BufferVert bv = {v, Ogre::Vector3(0.f, 0.f, 0.f), Ogre::Vector2(float(x) / (USize), float(y) / (VSize))};
          vertex_buffer.push_back(bv);

          // First part of quad.
          if (y != VSize - 1) {
            index_buffer.push_back((x - 1) + (y * USize));
            index_buffer.push_back(vertex_buffer.size() - 1);
            index_buffer.push_back((x - 1) + ((y + 1) * USize));
          }
          // Second part of quad.
          if (y != 0) {
            index_buffer.push_back((x - 1) + (y * USize));
            index_buffer.push_back(vertex_buffer.size() - 2);
            index_buffer.push_back(vertex_buffer.size() - 1);
          }
        }
      }
    }

    if (terrain_south_east) {
      pname.str(std::string());
      pname << ti->map_x + 1 << "_" << ti->map_y + 1;
      std::shared_ptr<l2p::UTexture> tm = std::dynamic_pointer_cast<l2p::UTexture>(terrain_south_east->GetObject(l2p::Package::GetName(pname.str())));
      if (tm && tm->mips.size() > 0) {
        const int x = USize;
        const int y = VSize;
        Ogre::Vector3 v( (x * scale.x) + translation.x
                       , (y * scale.y) + translation.y
                       , (tm->mips[0].getAs<l2p::ulittle16_t>()[0] * (scale.z / 256.f)) + translation.z
                       );
        box.merge(v);
        BufferVert bv = {v, Ogre::Vector3(0.f, 0.f, 0.f), Ogre::Vector2(float(x) / (USize), float(y) / (VSize))};
        vertex_buffer.push_back(bv);

        // First part of quad.
        if (y != VSize - 1) {
          index_buffer.push_back((x - 1) + ((y - 1) * USize));
          index_buffer.push_back(vertex_buffer.size() - 2);
          index_buffer.push_back(vertex_buffer.size() - 1);
        }
        // Second part of quad.
        if (y != 0) {
          index_buffer.push_back((x - 1) + ((y - 1) * USize));
          index_buffer.push_back(vertex_buffer.size() - 1);
          index_buffer.push_back(vertex_buffer.size() - (USize + 2));
        }
      }
    }
  }

  // Generate normals!
  for (unsigned int i = 0; i < index_buffer.size() / 3; ++i) {
    Ogre::Vector3 a = vertex_buffer[index_buffer[i * 3 + 1]].position - vertex_buffer[index_buffer[i * 3]].position;
    Ogre::Vector3 b = vertex_buffer[index_buffer[i * 3]].position - vertex_buffer[index_buffer[i * 3 + 2]].position;
    Ogre::Vector3 normal = b.crossProduct(a);
    normal.normalise();
    vertex_buffer[index_buffer[i * 3]].normal += normal;
    vertex_buffer[index_buffer[i * 3 + 1]].normal += normal;
    vertex_buffer[index_buffer[i * 3 + 2]].normal += normal;
  }

  for (unsigned int i = 0; i < vertex_buffer.size(); ++i) {
    vertex_buffer[i].normal.normalise();
  }

  Ogre::MeshPtr mesh = Ogre::MeshManager::getSingleton().createManual(Ogre::String(name) + Ogre::String(ti->name), "General");
  Ogre::VertexData  *data = new Ogre::VertexData();
  mesh->sharedVertexData = data;
  data->vertexCount = vertex_buffer.size();

  Ogre::VertexDeclaration *decl = data->vertexDeclaration;
  uint32_t offset = 0;
  decl->addElement(0, offset, Ogre::VET_FLOAT3, Ogre::VES_POSITION);
  offset += Ogre::VertexElement::getTypeSize(Ogre::VET_FLOAT3);
  decl->addElement(0, offset, Ogre::VET_FLOAT3, Ogre::VES_NORMAL);
  offset += Ogre::VertexElement::getTypeSize(Ogre::VET_FLOAT3);
  decl->addElement(0, offset, Ogre::VET_FLOAT2, Ogre::VES_TEXTURE_COORDINATES);
  offset += Ogre::VertexElement::getTypeSize(Ogre::VET_FLOAT2);

  Ogre::HardwareVertexBufferSharedPtr vbuf =
    Ogre::HardwareBufferManager::getSingleton().createVertexBuffer(
      offset,
      data->vertexCount,
      Ogre::HardwareBuffer::HBU_STATIC_WRITE_ONLY);
  vbuf->writeData(0, vbuf->getSizeInBytes(), &vertex_buffer.front(), true);
  data->vertexBufferBinding->setBinding(0, vbuf);

  // Setup index buffer.
  Ogre::HardwareIndexBufferSharedPtr ibuf =
    Ogre::HardwareBufferManager::getSingleton().createIndexBuffer(
      Ogre::HardwareIndexBuffer::IT_32BIT,
      index_buffer.size(),
      Ogre::HardwareBuffer::HBU_STATIC_WRITE_ONLY);

  ibuf->writeData(0, ibuf->getSizeInBytes(), &index_buffer.front(), true);
  Ogre::SubMesh *subMesh = mesh->createSubMesh();
  subMesh->useSharedVertices = true;
  subMesh->indexData->indexBuffer = ibuf;
  subMesh->indexData->indexCount  = index_buffer.size();
  subMesh->indexData->indexStart  = 0;

  mesh->_setBounds(box);
  mesh->_setBoundingSphereRadius((std::max(box.getMaximum().x - box.getMinimum().x, std::max(box.getMaximum().y - box.getMinimum().y, box.getMaximum().z - box.getMinimum().z))) / 2.0);

  mesh->load();

  Ogre::MaterialPtr mat = Ogre::MaterialManager::getSingleton().create(ti->GetPath(), "General");
  mat->getTechnique(0)->getPass(0)->createTextureUnitState()->setTexture(loadTexture(ti->Layers[0].Texture));

  Ogre::Entity *ent = mSceneMgr->createEntity(Ogre::String(name) + Ogre::String(ti->name) + "E", Ogre::String(name) + Ogre::String(ti->name));
  ent->setMaterial(mat);
  Ogre::SceneNode *node = mUnrealCordNode->createChildSceneNode();
  node->attachObject(ent);
}

struct SMV {
  Ogre::Vector3 position;
  Ogre::Vector3 normal;
  Ogre::Vector2 tex;
};

void TutorialApplication::loadStaticMeshActor(std::shared_ptr<l2p::AStaticMeshActor> sma) {
  Ogre::String smesh_name(sma->package->name);
  smesh_name += ".";
  smesh_name += sma->name;

  // Check to see if the mesh has already been loaded.
  if (Ogre::MeshManager::getSingleton().getByName(smesh_name, "General").isNull()) {
    std::vector<SMV> vertex_buffer;
    std::vector<uint16_t> index_buffer;

    std::shared_ptr<l2p::UStaticMesh> sm = sma->static_mesh;
    if (!sm)
      return;
    // Build vertex buffer.
    auto tc = sm->texture_coords[0].elements.begin();
    for (auto i = sm->vertices.begin(), e = sm->vertices.end(); i != e; ++i, ++tc) {
      SMV v = {ogre_cast(i->location), ogre_cast(i->normal), Ogre::Vector2(tc->u, tc->v)};
      vertex_buffer.push_back(v);
    }

    // Build index buffer.
    for (auto i = sm->surfaces.begin(), e = sm->surfaces.end(); i != e; ++i) {
      for (int tri = 0; tri < i->triangle_max; ++tri) {
        index_buffer.push_back(sm->vertex_indicies_1[i->index_offset + (tri * 3) + 2]);
        index_buffer.push_back(sm->vertex_indicies_1[i->index_offset + (tri * 3) + 1]);
        index_buffer.push_back(sm->vertex_indicies_1[i->index_offset + (tri * 3)]);
      }
    }

    Ogre::MeshPtr mesh = Ogre::MeshManager::getSingleton().createManual(smesh_name, "General");
    Ogre::VertexData  *data = new Ogre::VertexData();
    mesh->sharedVertexData = data;
    data->vertexCount = vertex_buffer.size();

    Ogre::VertexDeclaration *decl = data->vertexDeclaration;
    uint32_t offset = 0;
    decl->addElement(0, offset, Ogre::VET_FLOAT3, Ogre::VES_POSITION);
    offset += Ogre::VertexElement::getTypeSize(Ogre::VET_FLOAT3);
    decl->addElement(0, offset, Ogre::VET_FLOAT3, Ogre::VES_NORMAL);
    offset += Ogre::VertexElement::getTypeSize(Ogre::VET_FLOAT3);
    decl->addElement(0, offset, Ogre::VET_FLOAT2, Ogre::VES_TEXTURE_COORDINATES);
    offset += Ogre::VertexElement::getTypeSize(Ogre::VET_FLOAT2);

    Ogre::HardwareVertexBufferSharedPtr vbuf =
      Ogre::HardwareBufferManager::getSingleton().createVertexBuffer(
        offset,
        data->vertexCount,
        Ogre::HardwareBuffer::HBU_STATIC_WRITE_ONLY);
    vbuf->writeData(0, vbuf->getSizeInBytes(), &vertex_buffer.front(), true);
    data->vertexBufferBinding->setBinding(0, vbuf);

    // Setup index buffer.
    Ogre::HardwareIndexBufferSharedPtr ibuf =
      Ogre::HardwareBufferManager::getSingleton().createIndexBuffer(
        Ogre::HardwareIndexBuffer::IT_16BIT,
        index_buffer.size(),
        Ogre::HardwareBuffer::HBU_STATIC_WRITE_ONLY);

    ibuf->writeData(0, ibuf->getSizeInBytes(), &index_buffer.front(), true);
    for (auto i = sm->surfaces.begin(), e = sm->surfaces.end(); i != e; ++i) {
      l2p::SMeshMaterial &smm =
        sm->materials[std::distance(sm->surfaces.begin(), i)];
      if (i->triangle_max == 0 || (mIgnoreNonCollidable && !smm.enable_collision))
        continue;
      Ogre::SubMesh *subMesh = mesh->createSubMesh();
      // Setup geometry.
      subMesh->useSharedVertices = true;
      subMesh->indexData->indexBuffer = ibuf;
      subMesh->indexData->indexStart  = i->index_offset;
      subMesh->indexData->indexCount  = i->triangle_max * 3;

      // Setup material.
      std::shared_ptr<l2p::UMaterial> mat = smm.material;
      if (mat) {
        subMesh->setMaterialName(loadMaterial(mat)->getName());
      } else {
        std::stringstream ss;
        ss << sm->name.str() << ": " << "Failed to get material " << smm.material.package->GetObjectName(smm.material.index).str();
        Ogre::LogManager::getSingleton().logMessage(Ogre::LML_NORMAL, ss.str());
      }
    }

    mesh->updateMaterialForAllSubMeshes();

    Ogre::AxisAlignedBox box(ogre_cast(sm->bounding_box.min), ogre_cast(sm->bounding_box.max));
    mesh->_setBounds(box);
    mesh->_setBoundingSphereRadius((std::max(box.getMaximum().x - box.getMinimum().x, std::max(box.getMaximum().y - box.getMinimum().y, box.getMaximum().z - box.getMinimum().z))) / 2.0);

    mesh->load();
  }

  Ogre::String ent_name(sma->package->name);
  ent_name += ".";
  ent_name += sma->name;
  Ogre::Entity *ent = mSceneMgr->createEntity(ent_name, smesh_name, "General");
  ent->setRenderingDistance(ent->getBoundingRadius() * 75.f);
  Ogre::SceneNode *node = mUnrealCordNode->createChildSceneNode();
  node->attachObject(ent);
  assignActorPropsToNode(sma, node);
}

Ogre::MaterialPtr TutorialApplication::loadMaterial(std::shared_ptr<l2p::UMaterial> mat) {
  // See if we have already generated an Ogre material for this UE material.
  Ogre::MaterialPtr material = Ogre::MaterialManager::getSingleton().getByName(mat->GetPath());
  if (!material.isNull())
    return material;

  // Generate an Ogre material.
  material = Ogre::MaterialManager::getSingleton().create(mat->GetPath(), "General");
  if (std::shared_ptr<l2p::UShader> shader = std::dynamic_pointer_cast<l2p::UShader>(mat)) {
    Ogre::TexturePtr diffuse_map = loadTexture(shader->Diffuse);
    if (!diffuse_map.isNull()) {
      Ogre::Pass *pass = material->getTechnique(0)->getPass(0);
      pass->createTextureUnitState()->setTexture(diffuse_map);
      if (  shader->OutputBlending == l2p::UShader::OB_Masked
         || shader->AlphaTest) {
        pass->setAlphaRejectSettings(Ogre::CMPF_GREATER_EQUAL, shader->AlphaRef, true);
      }
      if (shader->TreatAsTwoSided || shader->TwoSided) {
        pass->setCullingMode(Ogre::CULL_NONE);
      }
      return material;
    }
  } else if (std::shared_ptr<l2p::UTexture> tex = std::dynamic_pointer_cast<l2p::UTexture>(mat)) {
    Ogre::TexturePtr diffuse_map = loadTexture(tex);
    if (!diffuse_map.isNull()) {
      Ogre::Pass *pass = material->getTechnique(0)->getPass(0);
      pass->createTextureUnitState()->setTexture(diffuse_map);
      if (tex->bAlphaTexture) {
        pass->setAlphaRejectSettings(Ogre::CMPF_GREATER_EQUAL, 128, true);
      }
      if (tex->bTwoSided) {
        pass->setCullingMode(Ogre::CULL_NONE);
      }
      return material;
    }
  } else if (std::shared_ptr<l2p::UFinalBlend> fb = std::dynamic_pointer_cast<l2p::UFinalBlend>(mat)) {
    Ogre::MaterialPtr back_mat = loadMaterial(fb->Material);
    if (!back_mat.isNull()) {
      material = back_mat->clone(fb->GetPath());
      material->setDepthWriteEnabled(fb->ZWrite);
      material->setDepthCheckEnabled(fb->ZTest);
      if (fb->AlphaTest)
        material->getTechnique(0)->getPass(0)->setAlphaRejectSettings(Ogre::CMPF_GREATER_EQUAL, fb->AlphaRef, true);
      if (fb->TreatAsTwoSided)
        material->setCullingMode(Ogre::CULL_NONE);
      switch (fb->FrameBufferBlending) {
      case l2p::UFinalBlend::FB_Overwrite:
        material->setSceneBlending(Ogre::SBT_REPLACE);
        break;
      case l2p::UFinalBlend::FB_Modulate:
        material->setSceneBlending(Ogre::SBT_MODULATE);
        break;
      case l2p::UFinalBlend::FB_AlphaBlend:
        material->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
        break;
      case l2p::UFinalBlend::FB_Add:
        material->setSceneBlending(Ogre::SBT_ADD);
        break;
      default:
        std::stringstream ss;
        ss << fb->GetPath() << ": " << "Unknown FrameBufferBlending " << fb->FrameBufferBlending;
        Ogre::LogManager::getSingleton().logMessage(Ogre::LML_NORMAL, ss.str());
        break;
      }
      return material;
    }
  }

  // Load fallback material.
  return Ogre::MaterialManager::getSingleton().getByName("StaticMesh/Checker");
}

struct Node {
  l2p::aligned_little32_t x;
  l2p::aligned_little32_t y;
  l2p::aligned_little32_t z;
  l2p::aligned_ulittle32_t links[8];
};

void TutorialApplication::loadPathnodeOff(l2p::StringRef path) {
  HANDLE hfile = ::CreateFileA( path.str().c_str()
                              , GENERIC_READ
                              , FILE_SHARE_READ
                              , 0
                              , OPEN_EXISTING
                              , FILE_FLAG_RANDOM_ACCESS
                              , 0
                              );
  if (hfile == INVALID_HANDLE_VALUE) {
    std::cerr << "CreateFile failed on " << path.str() << "\n";
    return;
  }
  HANDLE hmap = ::CreateFileMappingA( hfile
                                    , 0
                                    , PAGE_READONLY
                                    , 0
                                    , 0
                                    , 0
                                    );
  if (hmap == 0) {
    std::cerr << "CreateFileMapping failed on " << path.str() << "\n";
    ::CloseHandle(hfile);
    return;
  }
  char *data = (char*)::MapViewOfFile( hmap
                                     , FILE_MAP_READ
                                     , 0
                                     , 0
                                     , 0
                                     );
  if (data == nullptr) {
    std::cerr << "MapViewOfFile failed on " << path.str() << "\n";
    ::CloseHandle(hmap);
    ::CloseHandle(hfile);
    return;
  }

  // Get size of file.
  DWORD size = ::GetFileSize(hfile, 0);
  uint32_t node_count = size / sizeof(Node);
  Node *nodes = reinterpret_cast<Node*>(data);
  DynamicLines *lines = new DynamicLines(Ogre::RenderOperation::OT_LINE_LIST);
  for (Node *n = nodes + 1; n < nodes + node_count; ++n) {
    Ogre::Vector3 a(n->x, n->y, n->z);
    bool contains = false;
    for (auto i = mLoadedRegions.begin(), e = mLoadedRegions.end(); i != e; ++i) {
      if (getRegionAABB(i->first, i->second).contains(a)) {
        contains = true;
        break;
      }
    }
    if (!contains)
      continue;
    for (int i = 0; i < 8; ++i) {
      if (n->links[i] != 0) {
        Ogre::Vector3 b(nodes[n->links[i]].x, nodes[n->links[i]].y, nodes[n->links[i]].z);
        lines->addPoint(a);
        lines->addPoint(b);
      }
    }
  }
  lines->update();
  mUnrealCordNode->createChildSceneNode()->attachObject(lines);

  ::UnmapViewOfFile(data);
  ::CloseHandle(hmap);
  ::CloseHandle(hfile);
}

struct NodeL2J {
  l2p::big16_t z;
  l2p::aligned_little8_t links[8];
};

void TutorialApplication::loadPathnodeL2J(int regionX, int regionY) {
  fs::path path(mPathnodeL2JDir);
  std::stringstream ss;
  ss << regionX << "_" << regionY << ".pn";
  path /= ss.str();
  Ogre::Vector3 start = getRegionAABB(regionX, regionY).getMinimum();

  HANDLE hfile = ::CreateFileA( path.string().c_str()
                              , GENERIC_READ
                              , FILE_SHARE_READ
                              , 0
                              , OPEN_EXISTING
                              , FILE_FLAG_RANDOM_ACCESS
                              , 0
                              );
  if (hfile == INVALID_HANDLE_VALUE) {
    std::cerr << "CreateFile failed on " << path.string() << "\n";
    return;
  }
  HANDLE hmap = ::CreateFileMappingA( hfile
                                    , 0
                                    , PAGE_READONLY
                                    , 0
                                    , 0
                                    , 0
                                    );
  if (hmap == 0) {
    std::cerr << "CreateFileMapping failed on " << path.string() << "\n";
    ::CloseHandle(hfile);
    return;
  }
  char *data = (char*)::MapViewOfFile( hmap
                                     , FILE_MAP_READ
                                     , 0
                                     , 0
                                     , 0
                                     );
  if (data == nullptr) {
    std::cerr << "MapViewOfFile failed on " << path.string() << "\n";
    ::CloseHandle(hmap);
    ::CloseHandle(hfile);
    return;
  }

  // Build index;
  std::map<std::pair<int, int>, std::pair<int, NodeL2J*>> pathnode_data;
  for (int y = 0; y < 256; ++y) {
    for (int x = 0; x < 256; ++x) {
      int layers = *data++;
      pathnode_data[std::make_pair(x, y)] = std::make_pair(layers, reinterpret_cast<NodeL2J*>(data));
      data += layers * sizeof(NodeL2J);
    }
  }

  // Build line list.
  DynamicLines *lines = new DynamicLines(Ogre::RenderOperation::OT_LINE_LIST);
  for (int y = 0; y < 256; ++y) {
    for (int x = 0; x < 256; ++x) {
      // Get each node in this block.
      const auto &layers = pathnode_data[std::make_pair(x, y)];
      for (int i = 0; i < layers.first; ++i) {
        Ogre::Vector3 v(start.x + (x * 128) + 48, start.y + (y * 128) + 48, layers.second[i].z);
        // Get neighbors.
        int link_index, nx, ny;
        // North
        link_index = layers.second[i].links[0];
        if (link_index > 0) {
          --link_index;
          nx = x;
          ny = y - 1;
          auto ittr = pathnode_data.find(std::make_pair(nx, ny));
          if (ittr != pathnode_data.end()) {
            Ogre::Vector3 v2(start.x + (nx * 128) + 48, start.y + (ny * 128) + 48, ittr->second.second[link_index].z);
            lines->addPoint(v);
            lines->addPoint(v2);
          }
        }
        // North East
        link_index = layers.second[i].links[1];
        if (link_index > 0) {
          --link_index;
          nx = x + 1;
          ny = y - 1;
          auto ittr = pathnode_data.find(std::make_pair(nx, ny));
          if (ittr != pathnode_data.end()) {
            Ogre::Vector3 v2(start.x + (nx * 128) + 48, start.y + (ny * 128) + 48, ittr->second.second[link_index].z);
            lines->addPoint(v);
            lines->addPoint(v2);
          }
        }
        // East
        link_index = layers.second[i].links[2];
        if (link_index > 0) {
          --link_index;
          nx = x + 1;
          ny = y;
          auto ittr = pathnode_data.find(std::make_pair(nx, ny));
          if (ittr != pathnode_data.end()) {
            Ogre::Vector3 v2(start.x + (nx * 128) + 48, start.y + (ny * 128) + 48, ittr->second.second[link_index].z);
            lines->addPoint(v);
            lines->addPoint(v2);
          }
        }
        // South East
        link_index = layers.second[i].links[3];
        if (link_index > 0) {
          --link_index;
          nx = x + 1;
          ny = y + 1;
          auto ittr = pathnode_data.find(std::make_pair(nx, ny));
          if (ittr != pathnode_data.end()) {
            Ogre::Vector3 v2(start.x + (nx * 128) + 48, start.y + (ny * 128) + 48, ittr->second.second[link_index].z);
            lines->addPoint(v);
            lines->addPoint(v2);
          }
        }
        // South
        link_index = layers.second[i].links[4];
        if (link_index > 0) {
          --link_index;
          nx = x;
          ny = y + 1;
          auto ittr = pathnode_data.find(std::make_pair(nx, ny));
          if (ittr != pathnode_data.end()) {
            Ogre::Vector3 v2(start.x + (nx * 128) + 48, start.y + (ny * 128) + 48, ittr->second.second[link_index].z);
            lines->addPoint(v);
            lines->addPoint(v2);
          }
        }
        // South West
        link_index = layers.second[i].links[5];
        if (link_index > 0) {
          --link_index;
          nx = x - 1;
          ny = y + 1;
          auto ittr = pathnode_data.find(std::make_pair(nx, ny));
          if (ittr != pathnode_data.end()) {
            Ogre::Vector3 v2(start.x + (nx * 128) + 48, start.y + (ny * 128) + 48, ittr->second.second[link_index].z);
            lines->addPoint(v);
            lines->addPoint(v2);
          }
        }
        // West
        link_index = layers.second[i].links[6];
        if (link_index > 0) {
          --link_index;
          nx = x - 1;
          ny = y;
          auto ittr = pathnode_data.find(std::make_pair(nx, ny));
          if (ittr != pathnode_data.end()) {
            Ogre::Vector3 v2(start.x + (nx * 128) + 48, start.y + (ny * 128) + 48, ittr->second.second[link_index].z);
            lines->addPoint(v);
            lines->addPoint(v2);
          }
        }
        // North West
        link_index = layers.second[i].links[7];
        if (link_index > 0) {
          --link_index;
          nx = x - 1;
          ny = y - 1;
          auto ittr = pathnode_data.find(std::make_pair(nx, ny));
          if (ittr != pathnode_data.end()) {
            Ogre::Vector3 v2(start.x + (nx * 128) + 48, start.y + (ny * 128) + 48, ittr->second.second[link_index].z);
            lines->addPoint(v);
            lines->addPoint(v2);
          }
        }
      }
    }
  }
  lines->update();
  mUnrealCordNode->createChildSceneNode()->attachObject(lines);

  ::UnmapViewOfFile(data);
  ::CloseHandle(hmap);
  ::CloseHandle(hfile);
}

void TutorialApplication::loadGeodataL2J(int regionX, int regionY) {
  fs::path path(mGeodataL2JDir);
  std::stringstream ss;
  ss << regionX << "_" << regionY << ".l2j";
  path /= ss.str();
  Ogre::Vector3 start = getRegionAABB(regionX, regionY).getMinimum();

  HANDLE hfile = ::CreateFileA( path.string().c_str()
                              , GENERIC_READ
                              , FILE_SHARE_READ
                              , 0
                              , OPEN_EXISTING
                              , FILE_FLAG_RANDOM_ACCESS
                              , 0
                              );
  if (hfile == INVALID_HANDLE_VALUE) {
    std::cerr << "CreateFile failed on " << path.string() << "\n";
    return;
  }
  HANDLE hmap = ::CreateFileMappingA( hfile
                                    , 0
                                    , PAGE_READONLY
                                    , 0
                                    , 0
                                    , 0
                                    );
  if (hmap == 0) {
    std::cerr << "CreateFileMapping failed on " << path.string() << "\n";
    ::CloseHandle(hfile);
    return;
  }
  char *data = (char*)::MapViewOfFile( hmap
                                     , FILE_MAP_READ
                                     , 0
                                     , 0
                                     , 0
                                     );
  if (data == nullptr) {
    std::cerr << "MapViewOfFile failed on " << path.string() << "\n";
    ::CloseHandle(hmap);
    ::CloseHandle(hfile);
    return;
  }

  // Build point list.
  GeoCells *cell_list = new GeoCells(start.x, start.y);
  cell_list->setBoundingBox(getRegionAABB(regionX, regionY));
  for (int x = 0; x < 256; ++x) {
    for (int y = 0; y < 256; ++y) {
      uint8_t type = *data++;
      if (type == 0) { // Simple
        int16_t height = *reinterpret_cast<l2p::little16_t*>(data);
        cell_list->addCell(x * 8, y * 8, height, 0x0F, 0);
        data += 2;
      } else if (type == 1) { // Complex
        l2p::little16_t *cells = reinterpret_cast<l2p::little16_t*>(data);
        for (int cx = 0; cx < 8; ++cx) {
          for (int cy = 0; cy < 8; ++cy) {
            short height = *cells++;
            uint8_t nswe = height & 0x000f;
            height = (short)(height & 0xfff0);
            height = (short)(height >> 1);
            cell_list->addCell((x * 8) + cx, (y * 8) + cy, height, nswe, 1);
          }
        }
        data += 2 * 64;
      } else { // Multilayer
        for (int cx = 0; cx < 8; ++cx) {
          for (int cy = 0; cy < 8; ++cy) {
            uint8_t layers = *data++;
            for (int i = 0; i < layers; ++i) {
              short height = *reinterpret_cast<l2p::little16_t*>(data);
              uint8_t nswe = height & 0x000f;
              height = (short)(height & 0xfff0);
              height = (short)(height >> 1);
              cell_list->addCell((x * 8) + cx, (y * 8) + cy, height, nswe, 2);
              data += 2;
            }
          }
        }
      }
    }
  }
  cell_list->update();
  mUnrealCordNode->createChildSceneNode()->attachObject(cell_list);

  ::UnmapViewOfFile(data);
  ::CloseHandle(hmap);
  ::CloseHandle(hfile);
}

Ogre::AxisAlignedBox TutorialApplication::getRegionAABB(int x, int y) {
    int32_t minX = (x - 20) * 32768;
    int32_t minY = (y - 18) * 32768;
    return Ogre::AxisAlignedBox(
        Ogre::Vector3(static_cast<float>(minX), static_cast<float>(minY), -16383.f)
      , Ogre::Vector3(static_cast<float>(minX + 32768), static_cast<float>(minY + 32768), 16383.f)
      );
}

//-------------------------------------------------------------------------------------
void TutorialApplication::createScene(void)
{
  mUnrealCordNode = mSceneMgr->getRootSceneNode()->createChildSceneNode();
  mUnrealCordNode->setScale(-1.f, 1.f, 1.f);
  mUnrealCordNode->pitch(Ogre::Degree(-90.0f));
  mSceneMgr->setAmbientLight(Ogre::ColourValue(0.5f, 0.5f, 0.5f));
  Ogre::Light *l = mSceneMgr->createLight("MainLight");
  l->setType(Ogre::Light::LT_DIRECTIONAL);
  l->setPosition(0, 0, 0);
  l->setDirection(Ogre::Vector3(-0.45f, -0.75f, -0.55f).normalisedCopy());
  l->setDiffuseColour(0.5f, 0.5f, 0.5f);
  l->setSpecularColour(0.f, 0.f, 0.f);

  po::options_description desc("L2MapViewer");
  desc.add_options()
    ("help", "this help message")
    ("root", po::value<std::string>(), "L2 root directory")
    ("pathnode-off", po::value<std::string>(), "Path to the official pathnode file")
    ("pathnode-l2j", po::value<std::string>(), "Directory containing l2j pathnode files")
    ("geodata-l2j", po::value<std::string>(), "Directory containing l2j geodata files")
    ("ignore-non-collidable", po::value<bool>(), "Don't display non-collidable geometry")
    ("input", po::value<std::vector<std::string>>(), "input")
    ;

  po::positional_options_description p;
  p.add("input", -1);

  po::variables_map vm;
  po::store(po::command_line_parser(__argc, __argv).
            options(desc).positional(p).run(), vm);
  po::notify(vm);

  if (!vm.count("root")) {
    std::cerr << "Failed to find L2 root directory.\n";
  }

  l2p::Package::Initialize(vm["root"].as<std::string>());

  if (vm.count("ignore-non-collidable"))
    mIgnoreNonCollidable = true;
  else
    mIgnoreNonCollidable = false;

  // Load maps.
  if (vm.count("input")) {
    const std::vector<std::string> &input = vm["input"].as<std::vector<std::string>>();
    for (auto i = input.begin(), e = input.end(); i != e; ++i) {
      loadMap(*i);
    }
  }

  if (vm.count("pathnode-off")) {
    loadPathnodeOff(vm["pathnode-off"].as<std::string>());
  }

  if (vm.count("pathnode-l2j")) {
    mPathnodeL2JDir = vm["pathnode-l2j"].as<std::string>();
    for (auto i = mLoadedRegions.begin(), e = mLoadedRegions.end(); i != e; ++i) {
      loadPathnodeL2J(i->first, i->second);
    }
  }

  if (vm.count("geodata-l2j")) {
    const Ogre::RenderSystemCapabilities* caps = Ogre::Root::getSingleton().getRenderSystem()->getCapabilities();
    if (!caps->hasCapability(Ogre::RSC_GEOMETRY_PROGRAM)) {
      OGRE_EXCEPT(Ogre::Exception::ERR_NOT_IMPLEMENTED, "Your render system / hardware does not support geometry programs, "
        "so cannot load geodata. Sorry!",
                "TutorialApplication::createScene");
    }
    mGeodataL2JDir = vm["geodata-l2j"].as<std::string>();
    for (auto i = mLoadedRegions.begin(), e = mLoadedRegions.end(); i != e; ++i) {
      loadGeodataL2J(i->first, i->second);
    }
  }
}

bool BSPLineIntersectRecurse(l2p::UModel *m, const Ogre::Ray &ray, int node_index, Ogre::Vector3 &result) {
  l2p::BSPNode &n = m->nodes[node_index];
  Ogre::Plane plane(n.plane.X, n.plane.Y, n.plane.Z, n.plane.W);
  Ogre::Plane::Side side = plane.getSide(ray.getOrigin());

  int front_index;
  int back_index;

  // Get next node index based on side.
  if (side == Ogre::Plane::POSITIVE_SIDE) {
    front_index = n.front;
    back_index = n.back;
  } else if (side == Ogre::Plane::NEGATIVE_SIDE) {
    front_index = n.back;
    back_index = n.front;
  } else {
    // Touching the plane?
    result = ray.getOrigin();
    return true;
  }

  if (ray.intersects(plane).first) {
    // Check all nodes infront of this node if this node is in the ray path.
    if (front_index >= 0 && BSPLineIntersectRecurse(m, ray, front_index, result))
      return true;

    // Check this node.
    if (!ignoreNode(m, n, m->surfaces[n.surface])) {
      for (int verti = 2; verti < n.num_verticies; ++verti) {
        l2p::Vector a = m->points[m->vertexes[n.vert_pool].vertex],
                          b = m->points[m->vertexes[n.vert_pool + verti - 1].vertex],
                          c = m->points[m->vertexes[n.vert_pool + verti].vertex];
        auto res = Ogre::Math::intersects(ray,
          Ogre::Vector3(a.X, a.Y, a.Z),
          Ogre::Vector3(b.X, b.Y, b.Z),
          Ogre::Vector3(c.X, c.Y, c.Z));
        if (res.first) {
          result = res.second;
          return true;
        }
      }
    }
  }

  // Check all nodes behind this node.
  if (back_index >= 0 && BSPLineIntersectRecurse(m, ray, back_index, result))
      return true;

  return false;
}

Ogre::Vector3 BSPLineIntersect(l2p::UModel *m, Ogre::Ray ray) {
  // Transform ray into BSP space.
  Ogre::Matrix4 bspspace = Ogre::Matrix4::IDENTITY;
  bspspace.setScale(Ogre::Vector3(-50, 50, 50));
  Ogre::Quaternion bsprot(Ogre::Degree(-90.0f), Ogre::Vector3::UNIT_X);
  Ogre::Vector4 o(ray.getOrigin());
  Ogre::Vector4 d(ray.getDirection());
  o = o * bspspace * bsprot;
  d = d * bspspace * bsprot;

  ray.setOrigin(Ogre::Vector3(o.x, o.y, o.z));
  ray.setDirection(Ogre::Vector3(d.x, d.y, d.z).normalisedCopy());

  Ogre::Vector3 result;
  if (BSPLineIntersectRecurse(m, ray, 0, result))
    return result;
  else
    return ray.getOrigin();
}

bool TutorialApplication::mousePressed(const OIS::MouseEvent &arg, OIS::MouseButtonID id) {
  BaseApplication::mousePressed(arg, id);

  /*if (id == OIS::MB_Left) {
    Ogre::Ray cam_ray = mCamera->getCameraToViewportRay(0.5f, 0.5f);
    Ogre::RaySceneQuery *rayq = mSceneMgr->createRayQuery(cam_ray);
    Ogre::RaySceneQueryResult &result = rayq->execute();

    for (auto i = result.begin(), e = result.end(); i != e; ++i) {
      if (i->movable) {
        l2p::UModel *m = dynamic_cast<l2p::UModel*>(Ogre::any_cast<l2p::UObject*>(i->movable->getUserAny()));
        BSPLineIntersect(m, cam_ray);
      }
    }

    mSceneMgr->destroyQuery(rayq);
  }*/

  return true;
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
