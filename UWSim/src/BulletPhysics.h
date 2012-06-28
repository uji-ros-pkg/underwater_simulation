
#ifndef BULLETPHYSICS_H_
#define BULLETPHYSICS_H_

#include "SimulatorConfig.h"


#include <osgbDynamics/MotionState.h>
#include <osgbCollision/CollisionShapes.h>
#include <osgbCollision/Utils.h>

#include <btBulletDynamicsCommon.h>
#include <iostream>

#include <osgOcean/OceanScene>

//#include <osgbCollision/GLDebugDrawer.h>



#define UWSIM_DEFAULT_GRAVITY	btVector3(0,0,-1.0)

// Define filter groups
enum CollisionTypes {
    COL_NOTHING = 0x1 << 0,
    COL_OBJECTS = 0x1 << 1,
    COL_VEHICLE = 0x1 << 2,
};


class CollisionDataType : public osg::Referenced{
    public:
       CollisionDataType(std::string nam,std::string vehName,int isVehi){vehicleName=vehName;name=nam;isVehicle=isVehi;};
       std::string getObjectName(){if(isVehicle) return vehicleName; else return name;};
       std::string name, vehicleName;
       int isVehicle;
       
};

class BulletPhysics {

public:
	typedef enum {SHAPE_BOX, SHAPE_TRIMESH,SHAPE_COMPOUND_TRIMESH,SHAPE_COMPOUND_BOX} collisionShapeType_t;

	btDynamicsWorld * dynamicsWorld;
	//osgbCollision::GLDebugDrawer debugDrawer;

	BulletPhysics(double rotationOffset[3],double configGravity[3],osgOcean::OceanTechnique* oceanSurf);

	void setGravity(btVector3 g) {dynamicsWorld->setGravity( g );}

	btRigidBody* addDynamicObject(osg::MatrixTransform *root, osg::Node *node, btScalar mass, btVector3 inertia, collisionShapeType_t ctype, CollisionDataType * data );
	btRigidBody* addFloatingObject(osg::MatrixTransform *root, osg::Node *node, btScalar mass, btVector3 inertia, collisionShapeType_t ctype, CollisionDataType * data );
	btRigidBody* addKinematicObject(osg::MatrixTransform *root, osg::Node *node, btScalar mass, btVector3 inertia, collisionShapeType_t ctype, CollisionDataType * data);
	btRigidBody* addKinematicMirrorObject(osg::Node *node, btScalar mass, btVector3 inertia, collisionShapeType_t ctype,osg::Node * colShape , CollisionDataType * data );

	void stepSimulation(btScalar timeStep, int maxSubSteps, btScalar fixedTimeStep );
	void printManifolds();

	int getNumCollisions();

	btPersistentManifold * getCollision(int i);

      class MirrorTransformCallback : public osg::NodeCallback {
	private:
		btRigidBody* body;
	public:
		  MirrorTransformCallback(btRigidBody* b): osg::NodeCallback()
			{body=b;}

		  void operator() (osg::Node *node, osg::NodeVisitor *nv);
	};


	~BulletPhysics() {};

private:
	btDefaultCollisionConfiguration * collisionConfiguration;
	btCollisionDispatcher * dispatcher;
	btConstraintSolver * solver;
	btBroadphaseInterface * inter;
	std::vector<btRigidBody *> floatingBodies;
	std::vector<double> floatForces;
	osgOcean::OceanTechnique* oceanSurface;

	void processFloatingObjects();
	void cleanManifolds();
	btCollisionShape* GetCSFromOSG(osg::Node * node, collisionShapeType_t ctype);
	btRigidBody* addObject(osg::MatrixTransform *root, osg::Node *node, btScalar mass, btVector3 inertia, collisionShapeType_t ctype, CollisionDataType * data );

	
};


#endif

