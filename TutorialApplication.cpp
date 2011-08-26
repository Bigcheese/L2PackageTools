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

#include <vector>

//-------------------------------------------------------------------------------------
TutorialApplication::TutorialApplication(void)
{
}
//-------------------------------------------------------------------------------------
TutorialApplication::~TutorialApplication(void)
{
}

bool ignoreNode(L2Package::UModel *m, L2Package::BSPNode &n, L2Package::BSPSurface &s) {
  L2Package::Archive &ar = *m->archive;
  for (int i = 0; i < n.num_verticies; ++i) {
    Ogre::Vector3 loc = m->points[m->vertexes[n.vert_pool + i].vertex];
    Ogre::AxisAlignedBox box = m->getRegionAABB();
    if (!box.contains(loc))
      return true;
  }
  if (s.flags & L2Package::PF_WeDontCareAbout || !s.material)
    return true;
  if (  ar.GetClassName(s.material) == "g_01"
      || ar.GetClassName(s.material) == "SkybackgroundColor"
      || ar.GetClassName(s.material) == "Cloud_Final"
      || ar.GetClassName(s.material) == "HazeRing_Final"
      || ar.GetClassName(s.material) == "Base")
    return true;
  return false;
}

void TutorialApplication::loadMap(L2Package::StringRef path) {
  std::fstream file(path, std::ios::binary | std::ios::in);
  L2Package::l2_decrypt_streambuf l2ds(*file.rdbuf());
  std::istream package_stream(&l2ds);

  L2Package::Archive &ar = *new L2Package::Archive(package_stream);
  L2Package::ULevel *myLevel = ar.LoadExport<L2Package::ULevel>("myLevel");
  std::vector<L2Package::UModel*> models;
  ar.LoadExports("Model", models);

  // Find the largest model. The others aren't right or something like that...
  L2Package::UModel *m = nullptr;
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
    std::size_t loc = path.find_last_of("\\/");
    if (loc == path.npos)
      loc = 0;
    ++loc;
    path.substr(loc, 2).getAsInteger(10, m->regionX);
    path.substr(loc + 3, 2).getAsInteger(10, m->regionY);
  }

  if (m->points.size() == 0)
    return;

  // Shink model.
  std::vector<float> vertex_data;
  std::vector<uint32_t> index_buf;
  L2Package::Box bounds;

  // Build vertex and index buffer.
  for (auto ni = m->nodes.begin(), ne = m->nodes.end(); ni != ne; ++ni) {
    L2Package::BSPNode &n = *ni;
    L2Package::BSPSurface &s = m->surfaces[n.surface];

    if (ignoreNode(m, n, s))
      continue;

    uint32_t vert_start = vertex_data.size() / 6;

    // Vertex buffer.
    if (n.num_verticies > 0) {
      L2Package::Vector Normal = m->vectors[s.normal];

      for (uint32_t vert_index = 0; vert_index < n.num_verticies; ++vert_index) {
        const L2Package::Vector &pos = m->points[m->vertexes[n.vert_pool + vert_index].vertex];
        bounds += pos;
        vertex_data.push_back(pos.X);
        vertex_data.push_back(pos.Y);
        vertex_data.push_back(pos.Z);
        vertex_data.push_back(Normal.X);
        vertex_data.push_back(Normal.Y);
        vertex_data.push_back(Normal.Z);
      }

      if (s.flags & L2Package::PF_TwoSided) {
        for (uint32_t vert_index = 0; vert_index < n.num_verticies; ++vert_index) {
          const L2Package::Vector &pos = m->points[m->vertexes[n.vert_pool + vert_index].vertex];
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
    if (s.flags & L2Package::PF_TwoSided) {
      for (int verti = 2; verti < n.num_verticies; ++verti) {
        index_buf.push_back(vert_start);
        index_buf.push_back(vert_start + verti);
        index_buf.push_back(vert_start + verti - 1);
      }
    }
  }

  Ogre::MeshPtr mesh = Ogre::MeshManager::getSingleton().createManual(Ogre::String(path) + Ogre::String(m->name), "General");
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

  Ogre::Entity *ent = mSceneMgr->createEntity(Ogre::String(path) + Ogre::String(m->name) + "E", Ogre::String(path) + Ogre::String(m->name));
  ent->setUserAny(Ogre::Any(static_cast<L2Package::UObject*>(m)));
  Ogre::SceneNode *node = mUnrealCordNode->createChildSceneNode();
  node->attachObject(ent);
  node->showBoundingBox(true);
}

//-------------------------------------------------------------------------------------
void TutorialApplication::createScene(void)
{
  mUnrealCordNode = mSceneMgr->getRootSceneNode()->createChildSceneNode();
  mUnrealCordNode->setScale(-0.02f, 0.02f, 0.02f);
  mUnrealCordNode->pitch(Ogre::Degree(-90.0f));
  mSceneMgr->setAmbientLight(Ogre::ColourValue(0.5, 0.5, 0.5));
  Ogre::Light *l = mSceneMgr->createLight("MainLight");
  l->setPosition(20, 80, 50);

  loadMap("G:\\Program Files (x86)\\NCsoft\\Lineage II\\MAPS\\23_12.unr");
  loadMap("G:\\Program Files (x86)\\NCsoft\\Lineage II\\MAPS\\22_13.unr");
  loadMap("G:\\Program Files (x86)\\NCsoft\\Lineage II\\MAPS\\18_14.unr");
  loadMap("G:\\Program Files (x86)\\NCsoft\\Lineage II\\MAPS\\25_14.unr");
  loadMap("G:\\Program Files (x86)\\NCsoft\\Lineage II\\MAPS\\26_14.unr");
  loadMap("G:\\Program Files (x86)\\NCsoft\\Lineage II\\MAPS\\21_16.unr");
  loadMap("G:\\Program Files (x86)\\NCsoft\\Lineage II\\MAPS\\24_16.unr");
  loadMap("G:\\Program Files (x86)\\NCsoft\\Lineage II\\MAPS\\20_18.unr");
  loadMap("G:\\Program Files (x86)\\NCsoft\\Lineage II\\MAPS\\24_18.unr");
  loadMap("G:\\Program Files (x86)\\NCsoft\\Lineage II\\MAPS\\21_19.unr");
  loadMap("G:\\Program Files (x86)\\NCsoft\\Lineage II\\MAPS\\22_19.unr");
  loadMap("G:\\Program Files (x86)\\NCsoft\\Lineage II\\MAPS\\23_20.unr");
  loadMap("G:\\Program Files (x86)\\NCsoft\\Lineage II\\MAPS\\19_21.unr");
  loadMap("G:\\Program Files (x86)\\NCsoft\\Lineage II\\MAPS\\20_21.unr");
  loadMap("G:\\Program Files (x86)\\NCsoft\\Lineage II\\MAPS\\17_22.unr");
  loadMap("G:\\Program Files (x86)\\NCsoft\\Lineage II\\MAPS\\20_22.unr");
  loadMap("G:\\Program Files (x86)\\NCsoft\\Lineage II\\MAPS\\22_22.unr");
  loadMap("G:\\Program Files (x86)\\NCsoft\\Lineage II\\MAPS\\22_23.unr");
  loadMap("G:\\Program Files (x86)\\NCsoft\\Lineage II\\MAPS\\23_24.unr");
  loadMap("G:\\Program Files (x86)\\NCsoft\\Lineage II\\MAPS\\17_25.unr");
}

bool BSPLineIntersectRecurse(L2Package::UModel *m, const Ogre::Ray &ray, int node_index, Ogre::Vector3 &result) {
  L2Package::BSPNode &n = m->nodes[node_index];
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
        L2Package::Vector a = m->points[m->vertexes[n.vert_pool].vertex],
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

Ogre::Vector3 BSPLineIntersect(L2Package::UModel *m, Ogre::Ray ray) {
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

  if (id == OIS::MB_Left) {
    Ogre::Ray cam_ray = mCamera->getCameraToViewportRay(0.5f, 0.5f);
    Ogre::RaySceneQuery *rayq = mSceneMgr->createRayQuery(cam_ray);
    Ogre::RaySceneQueryResult &result = rayq->execute();

    for (auto i = result.begin(), e = result.end(); i != e; ++i) {
      if (i->movable) {
        L2Package::UModel *m = dynamic_cast<L2Package::UModel*>(Ogre::any_cast<L2Package::UObject*>(i->movable->getUserAny()));
        BSPLineIntersect(m, cam_ray);
      }
    }

    mSceneMgr->destroyQuery(rayq);
  }

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
