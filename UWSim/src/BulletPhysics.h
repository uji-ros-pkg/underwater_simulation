#include "SimulatorConfig.h"

#ifdef BUILD_BULLET_PHYSICS

#include <osgbBullet/MotionState.h>
#include <osgbBullet/CollisionShapes.h>
#include <osgbBullet/Utils.h>

#include <btBulletDynamicsCommon.h>

//#include <osgbCollision/GLDebugDrawer.h>

#ifndef BULLETPHYSICS_H
#define BULLETPHYSICS_H

#define UWSIM_DEFAULT_GRAVITY	btVector3(0,0,-1.0)

class BulletPhysics {

public:
	typedef enum {SHAPE_BOX, SHAPE_TRIMESH} collisionShapeType_t;

	btDynamicsWorld * dynamicsWorld;
	//osgbCollision::GLDebugDrawer debugDrawer;

	BulletPhysics();

	void setGravity(btVector3 g) {dynamicsWorld->setGravity( g );}

	btRigidBody* addDynamicObject(osg::MatrixTransform *root, osg::Node *node, btScalar mass, btVector3 inertia, collisionShapeType_t ctype );
	btRigidBody* addKinematicObject(osg::MatrixTransform *root, osg::Node *node, btScalar mass, btVector3 inertia, collisionShapeType_t ctype );
	btRigidBody* addKinematicMirrorObject(osg::MatrixTransform *root, osg::Node *node, btScalar mass, btVector3 inertia, collisionShapeType_t ctype );

	void stepSimulation(btScalar timeStep, int maxSubSteps=1, btScalar fixedTimeStep=btScalar(1.)/btScalar(60.) ) 
		{ //dynamicsWorld->debugDrawWorld();
		  dynamicsWorld->stepSimulation( timeStep, maxSubSteps, fixedTimeStep); }

	~BulletPhysics() {};

private:
	btDefaultCollisionConfiguration * collisionConfiguration;
	btCollisionDispatcher * dispatcher;
	btConstraintSolver * solver;
	btBroadphaseInterface * inter;

	btRigidBody* addObject(osg::MatrixTransform *root, osg::Node *node, btScalar mass, btVector3 inertia, collisionShapeType_t ctype );

	class MirrorTransformCallback : public osg::NodeCallback {
	private:
		btRigidBody* body;
	public:
		  MirrorTransformCallback(btRigidBody* b): osg::NodeCallback()
			{body=b;}

		  void operator() (osg::Node *node, osg::NodeVisitor *nv)  {
			OSG_INFO << "ToolPhysicsCallback should update hook transform" << std::endl;

			osg::Matrixd m;
			m = osg::computeLocalToWorld(nv->getNodePath() );
		
			traverse(node,nv);
			body->setWorldTransform( osgbBullet::asBtTransform(m) );
		  }
	};
};

BulletPhysics::BulletPhysics() {
    collisionConfiguration = new btDefaultCollisionConfiguration();
    dispatcher = new btCollisionDispatcher( collisionConfiguration );
    solver = new btSequentialImpulseConstraintSolver;

    btVector3 worldAabbMin( -10000, -10000, -10000 );
    btVector3 worldAabbMax( 10000, 10000, 10000 );
    inter = new btAxisSweep3( worldAabbMin, worldAabbMax, 1000 );

    dynamicsWorld = new btDiscreteDynamicsWorld( dispatcher, inter, solver, collisionConfiguration );

    dynamicsWorld->setGravity( UWSIM_DEFAULT_GRAVITY );
    //debugDrawer.setDebugMode(btIDebugDraw::DBG_DrawContactPoints|| btIDebugDraw::DBG_DrawWireframe || btIDebugDraw::DBG_DrawText);
    //dynamicsWorld->setDebugDrawer(&debugDrawer);
    //debugDrawer.BeginDraw();
    //debugDrawer.setEnabled(true);
}

btRigidBody* BulletPhysics::addObject(osg::MatrixTransform *root, osg::Node *node, btScalar mass, btVector3 inertia, collisionShapeType_t ctype ) {
    btCollisionShape* cs=NULL;

    if (ctype==SHAPE_BOX)
	cs= osgbBullet::btBoxCollisionShapeFromOSG( node );
    else if (ctype==SHAPE_TRIMESH)
	cs= osgbBullet::btTriMeshCollisionShapeFromOSG( node );

    osgbBullet::MotionState* motion = new osgbBullet::MotionState();
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
    //body->setActivationState( DISABLE_DEACTIVATION );
    dynamicsWorld->addRigidBody( body );

    return( body );
}

btRigidBody* BulletPhysics::addDynamicObject(osg::MatrixTransform *root, osg::Node *node, btScalar mass, btVector3 inertia, collisionShapeType_t ctype ) {
	return addObject(root, node, mass,inertia,ctype);
}

btRigidBody* BulletPhysics::addKinematicObject(osg::MatrixTransform *root, osg::Node *node, btScalar mass, btVector3 inertia, collisionShapeType_t ctype ) {
	btRigidBody *b=addObject(root,node, mass,inertia,ctype);
	if (b!=NULL) {
		b->setCollisionFlags( b->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT );
	}
	return b;
}

btRigidBody* BulletPhysics::addKinematicMirrorObject(osg::MatrixTransform *root, osg::Node *node, btScalar mass, btVector3 inertia, collisionShapeType_t ctype ) {
	btCollisionShape* cs=NULL;

	if (ctype==SHAPE_BOX)
		cs= osgbBullet::btBoxCollisionShapeFromOSG( node );
	else if (ctype==SHAPE_TRIMESH)
		cs= osgbBullet::btTriMeshCollisionShapeFromOSG( node );

	osgbBullet::MotionState* motion = new osgbBullet::MotionState();
	//osg::Matrix m=root->getMatrix();
	//motion->setTransform( root );
	//motion->setParentTransform( m );
	cs->calculateLocalInertia( mass, inertia );
	btRigidBody::btRigidBodyConstructionInfo rb( mass, motion, cs, inertia );
	btRigidBody* body = new btRigidBody( rb );
	body->setCollisionFlags( body->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT );
	dynamicsWorld->addRigidBody( body );

	node->setUpdateCallback(new MirrorTransformCallback(body));

	return body;
}

#endif

#endif //BUILD_BULLET_PHYSICS
