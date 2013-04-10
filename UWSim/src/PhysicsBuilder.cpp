/* 
 * Copyright (c) 2013 University of Jaume-I.
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the GNU Public License v3.0
 * which accompanies this distribution, and is available at
 * http://www.gnu.org/licenses/gpl.html
 * 
 * Contributors:
 *     Mario Prats
 *     Javier Perez
 */ 

#include "PhysicsBuilder.h"

PhysicsBuilder::PhysicsBuilder(SceneBuilder * scene_builder,ConfigFile config){
  loadPhysics(scene_builder,config);
}

void PhysicsBuilder::loadPhysics(SceneBuilder * scene_builder,ConfigFile config){

  physics = new BulletPhysics(config.gravity,scene_builder->scene->getOceanSurface(),config.physicsWater);
  OSG_INFO << "Loading Physics" << std::endl;

  //Add physics to vehicles
  for (unsigned int i=0; i<scene_builder->iauvFile.size();i++){
    for(unsigned int j=0; j<scene_builder->iauvFile[i]->urdf->link.size();j++){
      osg::Node * link =scene_builder->iauvFile[i]->urdf->link[j];
      osg::Node * cs= NULL;
      //Look for vehicle, and collision shapes on config.
      for(std::list<Vehicle>::iterator cfgVehicle=config.vehicles.begin();cfgVehicle!=config.vehicles.end();cfgVehicle++)
        if(cfgVehicle->name==scene_builder->iauvFile[i]->name)
	  for(unsigned int part=0;part<cfgVehicle->links.size();part++)
	    if(cfgVehicle->links[part].name==link->getName() && cfgVehicle->links[part].cs){
              //std::cout<<link->getName()<<" has cs"<<std::endl;
	      cs=UWSimGeometry::loadGeometry(cfgVehicle->links[part].cs);
	      if(!cs)
	        std::cerr<<"Collision shape couldn't load, using visual to create physics"<<std::endl;
	    }
      
      CollisionDataType * colData=new CollisionDataType(link->getName(),scene_builder->iauvFile[i]->name,1);
      physics->addKinematicObject(NULL,link,btScalar(1),btVector3(0,0,0), BulletPhysics::SHAPE_COMPOUND_BOX ,colData,cs);
      //TODO: Add node data type correctly(hand actuator).
      //NodeDataType * data= new NodeDataType(floorbody,0);
      //link->setUserData(data);
    }
  }

  //Add physics to objects  
  for(unsigned int i=0; i<scene_builder->objects.size();i++){
    //create Matrix Transform to use it on physics
    osg::Matrix mat;
    mat.makeIdentity();
    osg::MatrixTransform * mt= new osg::MatrixTransform();
    osg::Group * parent= scene_builder->objects[i]->getParent(0);
    mt->addChild(scene_builder->objects[i]);
    parent->removeChild(scene_builder->objects[i]);
    parent->addChild(mt);

    //Add physics to object
    btRigidBody *floorbody;
    //NodeDataType * data;
    CollisionDataType * colData=new CollisionDataType(scene_builder->objects[i]->getName()," ",0);

    //Init default physic properties
    double mass=1, inertia[3];
    memset(inertia,0,3*sizeof(double));
    BulletPhysics::collisionShapeType_t shape=BulletPhysics::SHAPE_BOX;  
    int customProp=0; //true if physic properties were added on xml.

    //Search for object in config, and look for physic properties
    for(std::list<Object>::iterator j=config.objects.begin();j!=config.objects.end();j++){
      if(j->name==scene_builder->objects[i]->getName() && j->physicProperties){
	customProp=1;
	mass=j->physicProperties->mass;
	inertia[0]=j->physicProperties->inertia[0];
	inertia[1]=j->physicProperties->inertia[1];
	inertia[2]=j->physicProperties->inertia[2];
	if(j->physicProperties->csType=="box")
	  shape=BulletPhysics::SHAPE_BOX;
	else if(j->physicProperties->csType=="sphere")
	  shape=BulletPhysics::SHAPE_SPHERE;
	else if(j->physicProperties->csType=="compound box")
	  shape=BulletPhysics::SHAPE_COMPOUND_BOX;
	else if(j->physicProperties->csType=="trimesh")
	  shape=BulletPhysics::SHAPE_TRIMESH;
	else if(j->physicProperties->csType=="compound trimesh")
	  shape=BulletPhysics::SHAPE_COMPOUND_TRIMESH;
	else
	  OSG_WARN << "Object: "<< j->name<<" has an unknown collision shape type: "<<j->physicProperties->csType<<". Using default box shape. Check xml file, allowed collision shapes are 'box' 'compound box' 'trimesh' 'compound trimesh'." << std::endl;
      }

    }

    //Objects called terrain will be added as concave trimesh shape and kinematic mode, objects with physic properties and simple shapes will be added with water physics and rest with physics.
    if(scene_builder->objects[i]->getName()=="terrain"){
      floorbody=physics->addKinematicObject(mt,scene_builder->objects[i],btScalar(0),btVector3(0,0,0), BulletPhysics::SHAPE_TRIMESH,colData);
      //data = new NodeDataType(floorbody,0);
    }
    else if(config.physicsWater.enable && customProp && (shape==BulletPhysics::SHAPE_BOX || shape==BulletPhysics::SHAPE_SPHERE)){
       physics->addFloatingObject(mt,scene_builder->objects[i],btScalar(mass),btVector3(inertia[0],inertia[1],inertia[2]), shape,colData);
    }
    else{
       physics->addDynamicObject(mt,scene_builder->objects[i],btScalar(mass),btVector3(inertia[0],inertia[1],inertia[2]), shape,colData);
      //data = new NodeDataType(flotante,1);
    }
    //wMb->setUserData(data); 
  }

  OSG_INFO << "Physics Loaded!" << std::endl;
}


