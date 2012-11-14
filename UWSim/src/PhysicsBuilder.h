#ifndef PHYSICSBUILDER_H
#define PHYSICSBUILDER_H

#include "SceneBuilder.h"
#include "BulletPhysics.h"
#include "ConfigXMLParser.h"

class PhysicsBuilder{
public:
  osg::ref_ptr<BulletPhysics> physics;
public:

  PhysicsBuilder(SceneBuilder * scene_builder,ConfigFile config);
  PhysicsBuilder(){};
  void loadPhysics(SceneBuilder * scene_builder,ConfigFile config);

};

#endif
