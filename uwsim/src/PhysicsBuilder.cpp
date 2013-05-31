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
      physics->addObject(NULL,link,colData,boost::shared_ptr<PhysicProperties>(),cs);
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

    //NodeDataType * data;
    CollisionDataType * colData=new CollisionDataType(scene_builder->objects[i]->getName()," ",0);


    //Search for object in config, and look for physic properties
    boost::shared_ptr<PhysicProperties> pp;
    for(std::list<Object>::iterator j=config.objects.begin();j!=config.objects.end();j++){
      if(j->name==scene_builder->objects[i]->getName()){
	pp=j->physicProperties;
      }

    }

    //Objects  objects with simple shapes will be added with water physics and rest with physics.
    if(config.physicsWater.enable && (pp->csType=="box" || pp->csType=="sphere")){
       physics->addFloatingObject(mt,scene_builder->objects[i],colData,pp);
    }
    else{
       physics->addObject(mt,scene_builder->objects[i],colData,pp);
    }
    //wMb->setUserData(data); 
  }

  OSG_INFO << "Physics Loaded!" << std::endl;

}


