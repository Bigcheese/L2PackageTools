/*
-----------------------------------------------------------------------------
Filename:    TutorialApplication.h
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
#ifndef __TutorialApplication_h_
#define __TutorialApplication_h_

#include "BaseApplication.h"
#undef GetObject
#include "l2p/StringRef.h"
#include "l2p/UObject.h"

class TutorialApplication : public BaseApplication
{
public:
  TutorialApplication(void);
  virtual ~TutorialApplication(void);

protected:
  void loadMap(l2p::StringRef path);
  Ogre::SceneNode *loadBSP(std::shared_ptr<l2p::UModel> m, bool ignoreNonVisible = true);
  void loadTerrain(std::shared_ptr<l2p::ATerrainInfo> ti);
  void loadStaticMeshActor(std::shared_ptr<l2p::AStaticMeshActor> sma);
  void loadPathnodeOff(l2p::StringRef path);
  void loadPathnodeL2J(int regionX, int regionY);
  void loadGeodataL2J(int regionX, int regionY);
  Ogre::MaterialPtr loadMaterial(std::shared_ptr<l2p::UMaterial> mat);
  Ogre::AxisAlignedBox getRegionAABB(int x, int y);
  virtual void createScene(void);
  virtual bool mousePressed(const OIS::MouseEvent &arg, OIS::MouseButtonID id);

  Ogre::SceneNode *mUnrealCordNode;
  std::string mPathnodeL2JDir;
  std::string mGeodataL2JDir;
  std::vector<std::pair<int, int>> mLoadedRegions;
  bool mIgnoreNonCollidable;
};

#endif // #ifndef __TutorialApplication_h_
