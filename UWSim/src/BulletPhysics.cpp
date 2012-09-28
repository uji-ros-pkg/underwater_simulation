
#include "BulletPhysics.h"

// Define filter masks
unsigned int vehicleCollidesWith( COL_OBJECTS );
unsigned int objectsCollidesWith( COL_OBJECTS | COL_VEHICLE );

void BulletPhysics::MirrorTransformCallback::operator() (osg::Node *node, osg::NodeVisitor *nv)  {
  OSG_INFO << "ToolPhysicsCallback should update hook transform" << std::endl;

  /* Bullet does not compute localizedWorld to move dynamicObjects, in order to check collisions localizedWorld is eliminated so
  collisions will be checked in different global coordinates but objects relative positions are OK, so objects should interact
  fine. */
  osg::NodePath nodepath=nv->getNodePath();
  osg::NodePath::iterator loc;
  int found=0;
  for(osg::NodePath::iterator i=nodepath.begin();i!=nodepath.end() && !found;++i){
    if( i[0]->getName()=="localizedWorld"){
      loc=i;
      found=1;
    }
  }
  nodepath.erase(loc);

  osg::Matrixd m;
  m = osg::computeLocalToWorld(nodepath );
		
  traverse(node,nv);
  body->getMotionState()->setWorldTransform( osgbCollision::asBtTransform(m) );
}

void BulletPhysics::stepSimulation(btScalar timeStep, int maxSubSteps=1, btScalar fixedTimeStep=btScalar(1.)/btScalar(60.) ) {
  //dynamicsWorld->debugDrawWorld();
  //printManifolds();
  cleanManifolds();
  processFloatingObjects();
  dynamicsWorld->stepSimulation( timeStep, maxSubSteps, fixedTimeStep);
}

void BulletPhysics::printManifolds(){
  std::cout<<dispatcher->getNumManifolds()<<std::endl;
  for(int i=0;i<dispatcher->getNumManifolds();i++){
    btCollisionObject* colObj0 =(btCollisionObject*) dispatcher->getManifoldByIndexInternal(i)->getBody0();
    btCollisionObject* colObj1 = (btCollisionObject*)dispatcher->getManifoldByIndexInternal(i)->getBody1();
    CollisionDataType * nombre=(CollisionDataType *)colObj0->getUserPointer();
    CollisionDataType * nombre2=(CollisionDataType *)colObj1->getUserPointer();
    double min=99999999999999999;
    for(int j=0;j<dispatcher->getManifoldByIndexInternal(i)->getNumContacts();j++)
	if(dispatcher->getManifoldByIndexInternal(i)->getContactPoint(j).getDistance() < min)
	  min=dispatcher->getManifoldByIndexInternal(i)->getContactPoint(j).getDistance();
    if(min<99999999999999999)
    std::cout<<i<<" "<<nombre->name<<" "<<" "<<nombre2->name<<" "<<min<<std::endl;
  }
}

BulletPhysics::BulletPhysics(double rotationOffset[3],double configGravity[3],osgOcean::OceanTechnique* oceanSurf) {
    collisionConfiguration = new btDefaultCollisionConfiguration();
    dispatcher = new btCollisionDispatcher( collisionConfiguration );
    solver = new btSequentialImpulseConstraintSolver;

    btVector3 worldAabbMin( -10000, -10000, -10000 );
    btVector3 worldAabbMax( 10000, 10000, 10000 );
    inter = new btAxisSweep3( worldAabbMin, worldAabbMax, 1000 );

    dynamicsWorld = new btDiscreteDynamicsWorld( dispatcher, inter, solver, collisionConfiguration );

    btVector3 gravity(configGravity[0],configGravity[1],configGravity[2]);
    if(configGravity[0]==0 && configGravity[1]==0 && configGravity[2]==0){
      gravity=UWSIM_DEFAULT_GRAVITY;
      gravity=gravity.rotate(btVector3(1,0,0),rotationOffset[0]);
      gravity=gravity.rotate(btVector3(0,1,0),rotationOffset[1]);
      gravity=gravity.rotate(btVector3(0,0,1),rotationOffset[2]);
    }

    dynamicsWorld->setGravity( gravity);
    floatingBodies.clear();
    floatForces.clear();
    oceanSurface=oceanSurf;
    /*debugDrawer.setDebugMode(btIDebugDraw::DBG_DrawContactPoints|| btIDebugDraw::DBG_DrawWireframe || btIDebugDraw::DBG_DrawText);
    dynamicsWorld->setDebugDrawer(&debugDrawer);
    debugDrawer.BeginDraw();
    debugDrawer.setEnabled(true);*/
}

btCollisionShape* BulletPhysics::GetCSFromOSG(osg::Node * node, collisionShapeType_t ctype){
    btCollisionShape* cs=NULL;

    if (ctype==SHAPE_BOX)
	cs= osgbCollision::btBoxCollisionShapeFromOSG(node);
    else if (ctype==SHAPE_COMPOUND_TRIMESH)
	cs= osgbCollision::btCompoundShapeFromOSGGeodes(node,CONVEX_TRIANGLEMESH_SHAPE_PROXYTYPE);
    else if (ctype==SHAPE_COMPOUND_BOX)
	cs= osgbCollision::btCompoundShapeFromOSGGeodes(node,BOX_SHAPE_PROXYTYPE);
    else if (ctype==SHAPE_TRIMESH)
	cs= osgbCollision::btTriMeshCollisionShapeFromOSG(node);

    return cs;
}

btRigidBody* BulletPhysics::addObject(osg::MatrixTransform *root, osg::Node *node, btScalar mass, btVector3 inertia, collisionShapeType_t ctype, CollisionDataType * data ) {
    

    btCollisionShape* cs=GetCSFromOSG( node, ctype);

    osgbDynamics::MotionState* motion = new osgbDynamics::MotionState();
    osg::Matrix m=root->getMatrix();
    motion->setTransform( root );
    //osg::Matrix m=osg::Matrixd::rotate(-M_PI_2,0,0,1);
    //m.setTrans(0.5,-2.15,-4.70);
    motion->setParentTransform( m );
    //btScalar mass( 0.5 );
    //btVector3 inertia( 0, 0, 0 );
    cs->calculateLocalInertia( mass, inertia );
    btRigidBody::btRigidBodyConstructionInfo rb( mass, motion, cs, inertia );
    btRigidBody* body = new btRigidBody( rb );
    body->setUserPointer(data);
    //body->setActivationState( DISABLE_DEACTIVATION );
    btBroadphaseProxy *bproxy = body->getBroadphaseHandle();
    if( bproxy ) {
      bproxy->m_collisionFilterGroup = short( COL_OBJECTS );
      bproxy->m_collisionFilterMask = short( objectsCollidesWith );
    } 
    dynamicsWorld->addRigidBody( body);


    return( body );
}

btRigidBody* BulletPhysics::addFloatingObject(osg::MatrixTransform *root, osg::Node *node, btScalar mass, btVector3 inertia, collisionShapeType_t ctype, CollisionDataType * data ) {

  btRigidBody* floating = addObject(root, node, mass,inertia,ctype,data);
  btVector3 min,max;
  floating->getAabb(min,max);
  max=max-min;
  double volume=max.x()*max.y()*max.z();

  floatingBodies.push_back(floating);
  floatForces.push_back(volume*1027);
  //std::cout<<"VOLUME: "<<volume<<" Empuje: "<<volume*1027<<std::endl;

  return floating;
}

btRigidBody* BulletPhysics::addDynamicObject(osg::MatrixTransform *root, osg::Node *node, btScalar mass, btVector3 inertia, collisionShapeType_t ctype, CollisionDataType * data ) {
	return addObject(root, node, mass,inertia,ctype,data);
}

btRigidBody* BulletPhysics::addKinematicObject(osg::MatrixTransform *root, osg::Node *node, btScalar mass, btVector3 inertia, collisionShapeType_t ctype, CollisionDataType * data) {
	btRigidBody *b=addObject(root,node, mass,inertia,ctype,data);
	if (b!=NULL) {
		b->setCollisionFlags( b->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT );
	}
	return b;
}

btRigidBody* BulletPhysics::addKinematicMirrorObject(osg::Node *node, btScalar mass, btVector3 inertia, collisionShapeType_t ctype,osg::Node * colShape=NULL, CollisionDataType * data=NULL ) {


	btCollisionShape* cs;
	if(colShape==NULL)
	  cs=GetCSFromOSG( node, ctype);
	else
	  cs=GetCSFromOSG( colShape, ctype);

	osgbDynamics::MotionState* motion = new osgbDynamics::MotionState();
	//osg::Matrix m=root->getMatrix();
	//motion->setTransform( root );
	//motion->setParentTransform( m );
	cs->calculateLocalInertia( mass, inertia );
	btRigidBody::btRigidBodyConstructionInfo rb( mass, motion, cs, inertia );
	btRigidBody* body = new btRigidBody( rb );
	body->setCollisionFlags( body->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT );
	btBroadphaseProxy *bproxy = body->getBroadphaseHandle();

	if( bproxy ) {
	  bproxy->m_collisionFilterGroup = short( COL_VEHICLE );
	  bproxy->m_collisionFilterMask = short( vehicleCollidesWith );
	} 
	dynamicsWorld->addRigidBody( body);
    	body->setUserPointer(data);
body->setActivationState( DISABLE_DEACTIVATION );

	node->setUpdateCallback(new MirrorTransformCallback(body));

	return body;
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

void BulletPhysics::processFloatingObjects(){

  for(unsigned int i=0;i<floatingBodies.size();i++){
    btRigidBody * floating=floatingBodies[i];
    btVector3 min,max;
    floating->getAabb(min,max);
    btVector3 velocidad=floating->getLinearVelocity();
    //std::cout<<"min:"<<min.x()<<" "<<min.y()<<" "<<min.z()<<" max: "<<max.x()<<" "<<max.y()<<" "<<max.z()<<std::endl;
    
    btVector3 med=(max+min)/2.0;
    std::cerr << "Pose in world: " << med.x() << " " << med.y() << " " << med.z() << std::endl;
    btVector3 alt=max-min;
    double Adn= alt.x()*alt.y(); //Area de la superficie normal a la direccion de movimiento
    double Ads= floatForces[i]/1027; //Volumen del objeto
    double Dn= pow(Adn/M_PI,0.5)*2,Ds=pow(Ads/M_PI*3/4.0,1/3.0)*2;
    double seaWaterViscosity=0.001792 *1000;
    double froz=3*M_PI*seaWaterViscosity*velocidad.z()*Dn*(1/3.0+(2/3.0)*(Ds/Dn));
    double fuerza=floatForces[i]*-1-froz,oceanSurf=oceanSurface->getSurfaceHeightAt(med.x(),med.y()) ;

    std::cout<<Adn<<" "<<Ads<<" "<<froz<<" "<<Dn<<" "<<Ds<<" "<<froz<<std::endl;

    //std::cout<<"altura: "<<alt.z()<<" Oceansurf: "<<oceanSurf<<" med: "<<med.z()<<std::endl;
    if(med.z()-oceanSurf < -alt.z())
	  fuerza=0;
    else if(med.z()-oceanSurf < alt.z())
	  fuerza=(med.z()-oceanSurf+alt.z())*fuerza-froz;
    //std::cout<<"fuerza: "<< fuerza<<" froz: "<<froz<<" velocidad: "<<velocidad.z()<<" Total:"<<fuerza-froz*velocidad.z()<<std::endl;
   floating->applyForce(btVector3(0,0,fuerza),btVector3(0,0,0));

    //std::cout<<oceanSurf<<" "<<max.z()<<std::endl;
  }

}
