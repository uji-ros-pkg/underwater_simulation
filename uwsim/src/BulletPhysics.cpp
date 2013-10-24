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

#include "BulletPhysics.h"


// Define filter masks
unsigned int vehicleCollidesWith( COL_OBJECTS);
unsigned int objectsCollidesWith( COL_EVERYTHING  );

class MyMotionState : public btMotionState {
public:

  MyMotionState(osg::Node * obj, osg::MatrixTransform *root){
    transf=root;
    object=obj;
  }

  void setNode(osg::Node *node) {
        object = node;
  }

  virtual void getWorldTransform(btTransform &worldTrans) const {
        worldTrans = osgbCollision::asBtTransform(*getWorldCoords(object));
  }

  virtual void setWorldTransform(const btTransform &worldTrans){
    //Object initial position
    osg::Matrixd * mat= getWorldCoords(transf->getParent(0));

    //Get object position in matrixd
    osg::Matrixd worldMatrix;
    btQuaternion rot = worldTrans.getRotation();
    btVector3 pos = worldTrans.getOrigin();
    worldMatrix.setTrans(osg::Vec3d(pos.x(),pos.y(),pos.z()));
    worldMatrix.setRotate(osg::Quat(rot.x(),rot.y(),rot.z(),rot.w()));
    
    //matrix transform from object initial position to final position
    osg::Matrixd rootmat=worldMatrix*(mat->inverse(*mat));

    //Apply transform to object matrix
    rootmat.setTrans(rootmat.getTrans());
    rootmat.setRotate(rootmat.getRotate());
    transf->setMatrix(rootmat);

  }

protected:
  osg::Node * object;
  osg::MatrixTransform *transf;

};

void BulletPhysics::stepSimulation(btScalar timeStep, int maxSubSteps=1, btScalar fixedTimeStep=btScalar(1.)/btScalar(60.) ) {
  //dynamicsWorld->debugDrawWorld();
  //printManifolds();
  //cleanManifolds();
  if(fluid)
    updateOceanSurface();
  ((btDynamicsWorld*)dynamicsWorld)->stepSimulation( timeStep, maxSubSteps, fixedTimeStep);
}

void BulletPhysics::printManifolds(){
  //std::cout<<dispatcher->getNumManifolds()<<std::endl;
  for(int i=0;i<dispatcher->getNumManifolds();i++){
    btCollisionObject* colObj0 =(btCollisionObject*) dispatcher->getManifoldByIndexInternal(i)->getBody0();
    btCollisionObject* colObj1 = (btCollisionObject*)dispatcher->getManifoldByIndexInternal(i)->getBody1();
    CollisionDataType * nombre=(CollisionDataType *)colObj0->getUserPointer();
    CollisionDataType * nombre2=(CollisionDataType *)colObj1->getUserPointer();
    double min=999999;
    for(int j=0;j<dispatcher->getManifoldByIndexInternal(i)->getNumContacts();j++)
	if(dispatcher->getManifoldByIndexInternal(i)->getContactPoint(j).getDistance() < min)
	  min=dispatcher->getManifoldByIndexInternal(i)->getContactPoint(j).getDistance();
    if(min<999999){
    std::cout<<i<<" ";
    if(nombre)
	std::cout<<nombre->name<<" "<<" ";
    if(nombre2)
	std::cout<<nombre2->name;
    std::cout<<" "<<min<<std::endl;

    }
  }
}

BulletPhysics::BulletPhysics(double configGravity[3],osgOcean::OceanTechnique* oceanSurf,PhysicsWater physicsWater) {
    collisionConfiguration = new btHfFluidRigidCollisionConfiguration();
    dispatcher = new btCollisionDispatcher( collisionConfiguration );
    solver = new btSequentialImpulseConstraintSolver();

    btVector3 worldAabbMin( -10000, -10000, -10000 );
    btVector3 worldAabbMax( 10000, 10000, 10000 );
    inter = new btAxisSweep3( worldAabbMin, worldAabbMax, 1000 );

    dynamicsWorld = new btHfFluidRigidDynamicsWorld( dispatcher, inter, solver, collisionConfiguration );
    dynamicsWorld->getDispatchInfo().m_enableSPU = true;

    btVector3 gravity(configGravity[0],configGravity[1],configGravity[2]);
    if(configGravity[0]==0 && configGravity[1]==0 && configGravity[2]==0){
      gravity=UWSIM_DEFAULT_GRAVITY;
    }

    dynamicsWorld->setGravity( gravity);
    oceanSurface=oceanSurf;

    if(physicsWater.enable){

    	fluid = new btHfFluid (physicsWater.resolution, physicsWater.size[0], physicsWater.size[1],physicsWater.size[2], physicsWater.size[3],physicsWater.size[4] , physicsWater.size[5]);
    	//fluid = new btHfFluid (btScalar(0.25), 100,100);
    	btTransform xform;
    	xform.setIdentity ();
    	xform.getOrigin() = btVector3(physicsWater.position[0], physicsWater.position[1], physicsWater.position[2]);
    	//xform.setRotation(btQuaternion(0,1.57,0));
    	fluid->setWorldTransform (xform);
    	fluid->setHorizontalVelocityScale (btScalar(0.0f));
    	fluid->setVolumeDisplacementScale (btScalar(0.0f));
    	dynamicsWorld->addHfFluid (fluid);

	for (int i = 0; i < fluid->getNumNodesLength()*fluid->getNumNodesWidth(); i++){
		fluid->setFluidHeight(i, btScalar(0.0f));
     	}

     	fluid->prep ();
    }
    else
	fluid=NULL;
    /*debugDrawer.setDebugMode(btIDebugDraw::DBG_DrawContactPoints|| btIDebugDraw::DBG_DrawWireframe || btIDebugDraw::DBG_DrawText);
    dynamicsWorld->setDebugDrawer(&debugDrawer);
    debugDrawer.BeginDraw();
    debugDrawer.setEnabled(true);*/
}

void BulletPhysics::updateOceanSurface(){

  int nodeswidth= fluid->getNumNodesWidth();

  int nodeslength= fluid->getNumNodesLength();
  double cellwidth= fluid->getGridCellWidth();
  double halfnodeswidth = nodeswidth/2*cellwidth;
  double halfnodeslength = nodeslength/2*cellwidth;

  for(int i=0;i<nodeswidth;i++){
    for(int j=0;j<nodeslength;j++){
      fluid->setFluidHeight(i,j,oceanSurface->getSurfaceHeightAt(i*cellwidth-halfnodeswidth+cellwidth/2,j*cellwidth-halfnodeslength+cellwidth/2)*-1);
    }
  }

}

btCollisionShape* BulletPhysics::GetCSFromOSG(osg::Node * node, collisionShapeType_t ctype){
    btCollisionShape* cs=NULL;

    if (ctype==SHAPE_BOX)
	cs= osgbCollision::btBoxCollisionShapeFromOSG(node);
    else if (ctype==SHAPE_SPHERE)
	cs= osgbCollision::btSphereCollisionShapeFromOSG(node);
    else if (ctype==SHAPE_COMPOUND_TRIMESH)
	cs= osgbCollision::btCompoundShapeFromOSGGeodes(node,CONVEX_TRIANGLEMESH_SHAPE_PROXYTYPE);
    else if (ctype==SHAPE_COMPOUND_BOX)
	cs= osgbCollision::btCompoundShapeFromOSGGeodes(node,BOX_SHAPE_PROXYTYPE);
    else if (ctype==SHAPE_COMPOUND_CYLINDER)
	cs= osgbCollision::btCompoundShapeFromOSGGeodes(node,CYLINDER_SHAPE_PROXYTYPE);
    else if (ctype==SHAPE_TRIMESH)
	cs= osgbCollision::btTriMeshCollisionShapeFromOSG(node);

    return cs;
}

btRigidBody* BulletPhysics::addObject(osg::MatrixTransform *root, osg::Node *node,CollisionDataType * data,boost::shared_ptr<PhysicProperties> pp,osg::Node * colShape){
   //if not physic properties set default.
   if(!pp){
     pp.reset(new PhysicProperties);
     pp->init();
     if(data->isVehicle){
	pp->isKinematic=1;
	pp->csType="compound box";
        pp->mass = 0; //there is no need to set mass (and inertia) to kinematic objects
     }
   }

   collisionShapeType_t ctype =SHAPE_BOX;

   if(pp->csType=="box")
     ctype=BulletPhysics::SHAPE_BOX;
   else if(pp->csType=="sphere")
     ctype=BulletPhysics::SHAPE_SPHERE;
   else if(pp->csType=="compound box")
     ctype=BulletPhysics::SHAPE_COMPOUND_BOX;
   else if(pp->csType=="compound cylinder")
     ctype=BulletPhysics::SHAPE_COMPOUND_CYLINDER;
   else if(pp->csType=="trimesh")
     ctype=BulletPhysics::SHAPE_TRIMESH;
   else if(pp->csType=="compound trimesh")
     ctype=BulletPhysics::SHAPE_COMPOUND_TRIMESH;
   else
     OSG_WARN << data->name<<" has an unknown collision shape type: "<<pp->csType<<". Using default box shape(dynamic) trimesh(kinematic). Check xml file, allowed collision shapes are 'box' 'compound box' 'trimesh' 'compound trimesh' 'compound cylinder." << std::endl;

   btCollisionShape* cs;
   if(colShape==NULL)
     cs=GetCSFromOSG( node, ctype);
   else
     cs=GetCSFromOSG( colShape, ctype);

    btVector3 inertia=btVector3(pp->inertia[0],pp->inertia[1],pp->inertia[2]);

    MyMotionState* motion = new MyMotionState(node,root);
   if (inertia.length()==0) //asking bullet to calculate inertia only if it is unset
    cs->calculateLocalInertia( pp->mass, inertia  );
    btRigidBody::btRigidBodyConstructionInfo rb( pp->mass, motion, cs,inertia);
    btRigidBody* body = new btRigidBody( rb );
    body->setUserPointer(data);

		
    if(pp->maxAngularLimit[0]>=pp->minAngularLimit[0] || pp->maxAngularLimit[1]>=pp->minAngularLimit[1] || pp->maxAngularLimit[1]>=pp->minAngularLimit[1] ||
	pp->maxLinearLimit[0]>=pp->minLinearLimit[0] || pp->maxLinearLimit[1]>=pp->minLinearLimit[1] || pp->maxLinearLimit[1]>=pp->minLinearLimit[1] ){
		btRigidBody* pBodyB = new btRigidBody(0,motion, 0);
		pBodyB->setActivationState(DISABLE_DEACTIVATION);

		btTransform frameInA, frameInB;
		frameInA = btTransform::getIdentity();
		frameInB = btTransform::getIdentity();
		if(pp->minAngularLimit[1]<pp->maxAngularLimit[1]){ //There is a constraint on Y axis (quaternions uncertainties can make it unstable)
		  if(not (pp->minAngularLimit[0]<pp->maxAngularLimit[0] or pp->minAngularLimit[2]<pp->maxAngularLimit[2])){ //There is no constraint on X and Z so we rotate to move it
		    frameInA.getBasis().setEulerZYX(0,0,1.57);
		    frameInB.getBasis().setEulerZYX(0,0,1.57);
		    double aux=pp->minAngularLimit[0];
		    pp->minAngularLimit[0]=pp->minAngularLimit[1];
		    pp->minAngularLimit[1]=aux;
		    aux=pp->maxAngularLimit[0];
		    pp->maxAngularLimit[0]=pp->maxAngularLimit[1];	
		    pp->maxAngularLimit[1]=aux;
		  }//TODO check other possible rotations to avoid aborting.
		  else if (pp->minAngularLimit[1]<=-1.50 || pp->minAngularLimit[1]>=1.50){ //safety threshold
		    std::cerr<<"Constraints in Y axis must be between -PI/2 PI/2"<<std::endl;
		    exit(0);
		  }
		}

		btGeneric6DofConstraint* pGen6DOF = new btGeneric6DofConstraint(*body, *pBodyB, frameInA, frameInB, true);
		pGen6DOF->setLinearLowerLimit(btVector3(pp->minLinearLimit[0], pp->minLinearLimit[1], pp->minLinearLimit[2]));
		pGen6DOF->setLinearUpperLimit(btVector3(pp->maxLinearLimit[0], pp->maxLinearLimit[1], pp->maxLinearLimit[2]));


		pGen6DOF->setAngularLowerLimit(btVector3(pp->minAngularLimit[0], pp->minAngularLimit[1], pp->minAngularLimit[2]));
		pGen6DOF->setAngularUpperLimit(btVector3(pp->maxAngularLimit[0], pp->maxAngularLimit[1], pp->maxAngularLimit[2]));

		dynamicsWorld->addConstraint(pGen6DOF, true);
	}

    //body->setLinearFactor(btVector3(pp->linearFactor[0],pp->linearFactor[1],pp->linearFactor[2]));
    //body->setAngularFactor(btVector3(pp->angularFactor[0],pp->angularFactor[1],pp->angularFactor[2]));

    body->setDamping(pp->linearDamping,pp->angularDamping);

    //addRigidBody adds its own collision masks, changing after object creation do not update masks so objects are removed and readded in order to update masks to improve collisions performance.
    dynamicsWorld->addRigidBody( body);
    if(data->isVehicle){
      dynamicsWorld->btCollisionWorld::removeCollisionObject(body);
      dynamicsWorld->addCollisionObject(body,short( COL_VEHICLE),short(vehicleCollidesWith));
    }
    else{
      dynamicsWorld->btCollisionWorld::removeCollisionObject(body);
      dynamicsWorld->addCollisionObject(body,short( COL_OBJECTS),short(objectsCollidesWith));
    }

    body->setActivationState( DISABLE_DEACTIVATION );
    if (pp->isKinematic) {
      body->setCollisionFlags( body->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT );
    } 


    return( body );
}

//Buoyant Shapes only admits simple convex  shapes
btConvexShape* BulletPhysics::GetConvexCSFromOSG(osg::Node * node, collisionShapeType_t ctype){
    btConvexShape* cs=NULL;

    if (ctype==SHAPE_BOX)
	cs= osgbCollision::btBoxCollisionShapeFromOSG(node);
    else if (ctype==SHAPE_SPHERE)
	cs= osgbCollision::btSphereCollisionShapeFromOSG(node);


    return cs;
}

btRigidBody* BulletPhysics::addFloatingObject(osg::MatrixTransform *root, osg::Node *node,CollisionDataType * data,boost::shared_ptr<PhysicProperties> pp,osg::Node * colShape){
   if(!pp){
     pp.reset(new PhysicProperties);
     pp->init();
   }

   collisionShapeType_t ctype =SHAPE_BOX;

   if(pp->csType=="box")
     ctype=BulletPhysics::SHAPE_BOX;
   else if(pp->csType=="sphere")
     ctype=BulletPhysics::SHAPE_SPHERE;
   else
     OSG_WARN << data->name<<" has an unknown collision shape type: "<<pp->csType<<". Using default box shape(dynamic) trimesh(kinematic). Check xml file, allowed collision shapes are 'box' 'compound box' 'trimesh' 'compound trimesh'." << std::endl;



   btConvexShape* cs;
   if(colShape==NULL)
     cs=GetConvexCSFromOSG( node, ctype);
   else
     cs=GetConvexCSFromOSG( colShape, ctype);

    btVector3 inertia=btVector3(pp->inertia[0],pp->inertia[1],pp->inertia[2]);


    MyMotionState* motion = new MyMotionState(node,root);
    cs->calculateLocalInertia( pp->mass, inertia );

    btHfFluidBuoyantConvexShape* buoyantShape = new btHfFluidBuoyantConvexShape(cs);
    buoyantShape->generateShape (btScalar(0.05f), btScalar(0.01f));
    buoyantShape->setFloatyness (btScalar(1.0f));

    btRigidBody::btRigidBodyConstructionInfo rb( pp->mass, motion, buoyantShape, inertia );
    btRigidBody* body= new btRigidBody( rb );

    body->setUserPointer(data);

    //body->setLinearFactor(btVector3(pp->linearFactor[0],pp->linearFactor[1],pp->linearFactor[2]));
    //body->setAngularFactor(btVector3(pp->angularFactor[0],pp->angularFactor[1],pp->angularFactor[2]));

    body->setDamping(pp->linearDamping,pp->angularDamping);

    //addRigidBody adds its own collision masks, changing after object creation do not update masks so objects are removed and readded in order to update masks to improve collisions performance.
      dynamicsWorld->addRigidBody( body);
      dynamicsWorld->btCollisionWorld::removeCollisionObject(body);
      dynamicsWorld->addCollisionObject(body,short( COL_OBJECTS),short(objectsCollidesWith));
 
    body->setActivationState( DISABLE_DEACTIVATION );
    return( body );
}


int BulletPhysics::getNumCollisions(){
  return dispatcher->getNumManifolds();
}

btPersistentManifold * BulletPhysics::getCollision(int i){
  return dispatcher->getManifoldByIndexInternal(i);
}


void BulletPhysics::cleanManifolds(){  //it removes contact points with long lifetime
  //std::cout<<dispatcher->getNumManifolds()<<"aa"<<std::endl;
  for(int i=0;i<dispatcher->getNumManifolds();i++){
    btPersistentManifold * col = dispatcher->getManifoldByIndexInternal(i);
    //std::cout<<col->getNumContacts()<<std::endl;
    for(int j=0;j < col->getNumContacts();j++)
	if(col->getContactPoint(j).getLifeTime() > 300)
	  col->removeContactPoint(j);

  }	  
}
