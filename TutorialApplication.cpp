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
#undef GetObject
#include "l2p/UObject.h"

#include <vector>

Ogre::Vector3 ogre_cast(const l2p::Vector &v) {
  return Ogre::Vector3(v.X, v.Y, v.Z);
}

Ogre::AxisAlignedBox ogre_cast(const l2p::Box &b) {
  return Ogre::AxisAlignedBox(ogre_cast(b.min), ogre_cast(b.max));
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
  l2p::Package &p = *m->package;
  for (int i = 0; i < n.num_verticies; ++i) {
    Ogre::Vector3 loc = ogre_cast(m->points[m->vertexes[n.vert_pool + i].vertex]);
    Ogre::AxisAlignedBox box = ogre_cast(m->getRegionAABB());
    if (!box.contains(loc))
      return true;
  }
  if (s.flags & l2p::PF_WeDontCareAbout || !s.material.index)
    return true;
  return false;
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

  // Get region from path.
  {
    name.substr(0, 2).getAsInteger(10, m->regionX);
    name.substr(3, 2).getAsInteger(10, m->regionY);
  }

  if (m->points.size() != 0)
    loadBSP(m);

  std::vector<std::shared_ptr<l2p::ATerrainInfo>> terrains;
  package->GetObjects("TerrainInfo", terrains);

  for (auto i = terrains.begin(), e = terrains.end(); i != e; ++i) {
    loadTerrain(*i);
  }

  std::vector<std::shared_ptr<l2p::AStaticMeshActor>> smeshes;
  package->GetObjects("StaticMeshActor", smeshes);

  for (auto i = smeshes.begin(), e = smeshes.end(); i != e; ++i) {
    loadStaticMeshActor(*i);
  }
}

void TutorialApplication::loadBSP(std::shared_ptr<l2p::UModel> m) {
  l2p::Name name = m->package->name;
  std::vector<float> vertex_data;
  std::vector<uint32_t> index_buf;
  l2p::Box bounds;

  // Build vertex and index buffer.
  for (auto ni = m->nodes.begin(), ne = m->nodes.end(); ni != ne; ++ni) {
    l2p::BSPNode &n = *ni;
    l2p::BSPSurface &s = m->surfaces[n.surface];

    if (ignoreNode(m.get(), n, s))
      continue;

    uint32_t vert_start = vertex_data.size() / 6;

    // Vertex buffer.
    if (n.num_verticies > 0) {
      l2p::Vector Normal = m->vectors[s.normal];

      for (uint32_t vert_index = 0; vert_index < n.num_verticies; ++vert_index) {
        const l2p::Vector &pos = m->points[m->vertexes[n.vert_pool + vert_index].vertex];
        bounds += pos;
        vertex_data.push_back(pos.X);
        vertex_data.push_back(pos.Y);
        vertex_data.push_back(pos.Z);
        vertex_data.push_back(Normal.X);
        vertex_data.push_back(Normal.Y);
        vertex_data.push_back(Normal.Z);
      }

      if (s.flags & l2p::PF_TwoSided) {
        for (uint32_t vert_index = 0; vert_index < n.num_verticies; ++vert_index) {
          const l2p::Vector &pos = m->points[m->vertexes[n.vert_pool + vert_index].vertex];
          vertex_data.push_back(pos.X);
          vertex_data.push_back(pos.Y);
          vertex_data.push_back(pos.Z);
          vertex_data.push_back(Normal.X);
          vertex_data.push_back(Normal.Y);
          vertex_data.push_back(-Normal.Z);
        }
      }
    }

    // Index buffer.
    for (int verti = 2; verti < n.num_verticies; ++verti) {
      index_buf.push_back(vert_start);
      index_buf.push_back(vert_start + verti - 1);
      index_buf.push_back(vert_start + verti);
    }
    if (s.flags & l2p::PF_TwoSided) {
      for (int verti = 2; verti < n.num_verticies; ++verti) {
        index_buf.push_back(vert_start);
        index_buf.push_back(vert_start + verti);
        index_buf.push_back(vert_start + verti - 1);
      }
    }
  }

  if (vertex_data.size() == 0 || index_buf.size() == 0)
    return;

  Ogre::MeshPtr mesh = Ogre::MeshManager::getSingleton().createManual(Ogre::String(name) + Ogre::String(m->name), "General");
  Ogre::VertexData  *data = new Ogre::VertexData();
  mesh->sharedVertexData = data;
  data->vertexCount = vertex_data.size() / 6;

  Ogre::VertexDeclaration *decl = data->vertexDeclaration;
  uint32_t offset = 0;
  decl->addElement(0, offset, Ogre::VET_FLOAT3, Ogre::VES_POSITION);
  offset += Ogre::VertexElement::getTypeSize(Ogre::VET_FLOAT3);
  decl->addElement(0, offset, Ogre::VET_FLOAT3, Ogre::VES_NORMAL);
  offset += Ogre::VertexElement::getTypeSize(Ogre::VET_FLOAT3);

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
      Ogre::HardwareIndexBuffer::IT_32BIT,
      index_buf.size(),
      Ogre::HardwareBuffer::HBU_STATIC_WRITE_ONLY);

  ibuf->writeData(0, ibuf->getSizeInBytes(), &index_buf.front(), true);
  Ogre::SubMesh *subMesh = mesh->createSubMesh();
  subMesh->useSharedVertices = true;
  subMesh->indexData->indexBuffer = ibuf;
  subMesh->indexData->indexCount  = index_buf.size();
  subMesh->indexData->indexStart  = 0;

  mesh->_setBounds(Ogre::AxisAlignedBox(bounds.min.X, bounds.min.Y, bounds.min.Z, bounds.max.X, bounds.max.Y, bounds.max.Z));
  mesh->_setBoundingSphereRadius((std::max(bounds.max.X - bounds.min.X, std::max(bounds.max.Y - bounds.min.Y, bounds.max.Z - bounds.min.Z))) / 2.0);

  mesh->load();

  Ogre::Entity *ent = mSceneMgr->createEntity(Ogre::String(name) + Ogre::String(m->name) + "E", Ogre::String(name) + Ogre::String(m->name));
  ent->setCastShadows(true);
  ent->setUserAny(Ogre::Any(static_cast<l2p::UObject*>(m.get())));
  Ogre::SceneNode *node = mUnrealCordNode->createChildSceneNode();
  node->attachObject(ent);
}

struct BufferVert {
  Ogre::Vector3 position;
  Ogre::Vector3 normal;
  Ogre::RGBA color;
};

void TutorialApplication::loadTerrain(std::shared_ptr<l2p::ATerrainInfo> ti) {
  l2p::Name name = ti->package->name;
  std::vector<BufferVert> vertex_buffer;
  std::vector<Ogre::RGBA> color_buffer;
  std::vector<uint32_t> index_buffer;
  std::vector<uint16_t> &height_map = ti->terrain_map->G16_data;
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
  color_buffer.reserve(USize * VSize);
  Ogre::AxisAlignedBox box;
  Ogre::RenderSystem *rs = Ogre::Root::getSingleton().getRenderSystem();
  float sealevel = -3770;
  for (int y = 0; y < VSize; ++y) {
    for (int x = 0; x < USize; ++x) {
      Ogre::Vector3 v( (x * scale.x) + translation.x
                     , (y * scale.y) + translation.y
                     , (height_map[(y * USize) + x] * (scale.z / 256.f)) + translation.z
                     );
      box.merge(v);
      BufferVert bv = {v, Ogre::Vector3(0.f, 0.f, 0.f), 0};
      float val = -((v.z - min_hight)/(min_hight - max_hight));
      Ogre::ColourValue cv;
      if (v.z < sealevel)
        cv = Ogre::ColourValue(0.f, 0.f, 1.f) * val;
      else
        cv = Ogre::ColourValue(0.f, 1.f, 0.f) * val;
      rs->convertColourValue(cv, &bv.color);
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

  // Generate normals!
  for (int i = 0; i < index_buffer.size() / 3; ++i) {
    Ogre::Vector3 a = vertex_buffer[index_buffer[i * 3 + 1]].position - vertex_buffer[index_buffer[i * 3]].position;
    Ogre::Vector3 b = vertex_buffer[index_buffer[i * 3]].position - vertex_buffer[index_buffer[i * 3 + 2]].position;
    Ogre::Vector3 normal = b.crossProduct(a);
    normal.normalise();
    vertex_buffer[index_buffer[i * 3]].normal += normal;
    vertex_buffer[index_buffer[i * 3 + 1]].normal += normal;
    vertex_buffer[index_buffer[i * 3 + 2]].normal += normal;
  }

  for (int i = 0; i < vertex_buffer.size(); ++i) {
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
  decl->addElement(0, offset, Ogre::VET_COLOUR, Ogre::VES_DIFFUSE);
  offset += Ogre::VertexElement::getTypeSize(Ogre::VET_COLOUR);

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

  Ogre::MaterialPtr mat = Ogre::MaterialManager::getSingleton().create(
    "Test/VertColor",
    Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
  mat->getTechnique(0)->getPass(0)->setVertexColourTracking(Ogre::TVC_AMBIENT);

  Ogre::Entity *ent = mSceneMgr->createEntity(Ogre::String(name) + Ogre::String(ti->name) + "E", Ogre::String(name) + Ogre::String(ti->name));
  ent->setMaterialName("Test/VertColor");
  Ogre::SceneNode *node = mUnrealCordNode->createChildSceneNode();
  node->attachObject(ent);
}

struct SMV {
  Ogre::Vector3 position;
  Ogre::Vector3 normal;
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
    for (auto i = sm->vertices.begin(), e = sm->vertices.end(); i != e; ++i) {
      SMV v = {ogre_cast(i->location), ogre_cast(i->normal)};
      vertex_buffer.push_back(v);
    }

    // Build index buffer.
    int tri_offset = 0;
    for (auto i = sm->surfaces.begin(), e = sm->surfaces.end(); i != e; ++i) {
      for (int tri = 0; tri < i->triangle_count; ++tri) {
        if (tri_offset >= sm->vertex_indicies_1.size())
          break;
        index_buffer.push_back(sm->vertex_indicies_1[tri_offset + 2]);
        index_buffer.push_back(sm->vertex_indicies_1[tri_offset + 1]);
        index_buffer.push_back(sm->vertex_indicies_1[tri_offset]);
        tri_offset += 3;
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
    Ogre::SubMesh *subMesh = mesh->createSubMesh();
    subMesh->useSharedVertices = true;
    subMesh->indexData->indexBuffer = ibuf;
    subMesh->indexData->indexCount  = index_buffer.size();
    subMesh->indexData->indexStart  = 0;

    Ogre::AxisAlignedBox box(ogre_cast(sm->bounding_box.min), ogre_cast(sm->bounding_box.max));
    mesh->_setBounds(box);
    mesh->_setBoundingSphereRadius((std::max(box.getMaximum().x - box.getMinimum().x, std::max(box.getMaximum().y - box.getMinimum().y, box.getMaximum().z - box.getMinimum().z))) / 2.0);

    mesh->load();
  }

  Ogre::String ent_name(sma->package->name);
  ent_name += ".";
  ent_name += sma->name;
  Ogre::Entity *ent = mSceneMgr->createEntity(ent_name, smesh_name, "General");
  Ogre::SceneNode *node = mUnrealCordNode->createChildSceneNode();
  node->attachObject(ent);
  node->setPosition(sma->location.X, sma->location.Y, sma->location.Z);
  node->pitch(Ogre::Radian(0.000096f) * sma->rotation.pitch);
  node->roll(Ogre::Radian(-0.000096f) * sma->rotation.yaw);
  node->yaw(Ogre::Radian(-0.000096f) * sma->rotation.roll);
  node->setScale(ogre_cast(sma->draw_scale_3d));
}

//-------------------------------------------------------------------------------------
void TutorialApplication::createScene(void)
{
  mUnrealCordNode = mSceneMgr->getRootSceneNode()->createChildSceneNode();
  mUnrealCordNode->setScale(-1.f, 1.f, 1.f);
  mUnrealCordNode->pitch(Ogre::Degree(-90.0f));
  mSceneMgr->setAmbientLight(Ogre::ColourValue(0.5, 0.5, 0.5));
  mSceneMgr->setShadowTechnique(Ogre::SHADOWTYPE_TEXTURE_MODULATIVE);
  Ogre::Light *l = mSceneMgr->createLight("MainLight");
  l->setType(Ogre::Light::LT_DIRECTIONAL);
  l->setPosition(0, 0, 0);
  l->setDirection(Ogre::Vector3(-0.5, -0.75, -0.5).normalisedCopy());
  l->setDiffuseColour(0.5f, 0.5f, 0.5f);
  l->setSpecularColour(0.f, 0.f, 0.f);

  if (__argc < 2)
    return;

  l2p::Package::Initialize(__argv[1]);

  // Load maps.
  for (int i = 2; i < __argc; ++i) {
    loadMap(__argv[i]);
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
