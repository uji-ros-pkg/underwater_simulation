/*
BuoyantShape vs ConvexSHape collision algorithm added to Experimental Buoyancy fluid demo.
*/
#include <stdio.h>

#include <uwsim/BulletHfFluid/btHfFluidBuoyantShapeCollisionAlgorithm.h>
#include <uwsim/BulletHfFluid/BuoyantShapeConvexCollisionAlgorithm.h>
#include <uwsim/BulletHfFluid/btHfFluidBuoyantConvexShape.h>
#include <BulletCollision/CollisionDispatch/btCollisionDispatcher.h>
#include <BulletCollision/CollisionShapes/btSphereShape.h>
#include <BulletCollision/CollisionShapes/btBoxShape.h>

#include <BulletCollision/CollisionDispatch/btCollisionObject.h>
#include <BulletDynamics/Dynamics/btRigidBody.h>
#include <uwsim/BulletHfFluid/btHfFluid.h>

#include <iostream>

BuoyantShapeConvexCollisionAlgorithm::BuoyantShapeConvexCollisionAlgorithm(const btCollisionAlgorithmConstructionInfo& ci,btCollisionObject* col0,btCollisionObject* col1,btSimplexSolverInterface* simplexSolver, btConvexPenetrationDepthSolver* pdSolver, bool isSwapped,int proxyType)
: btCollisionAlgorithm(ci), m_isSwapped(isSwapped), m_proxyType(proxyType)
{

	if(!isSwapped){
	  m_collisionObject0 = col0;
	  m_collisionObject1 = col1;
	}
	else{
	  m_collisionObject0 = col1;
	  m_collisionObject1 = col0;
	}

	if(proxyType==COMPOUND_SHAPE_PROXYTYPE)
	  m_collisionAlgorithm= new btCompoundCollisionAlgorithm( ci,  m_collisionObject0, m_collisionObject1,isSwapped);
	else if(proxyType==TRIANGLE_MESH_SHAPE_PROXYTYPE)
	  m_collisionAlgorithm= new btConvexConcaveCollisionAlgorithm( ci,   m_collisionObject1, m_collisionObject0,isSwapped);
	else if(proxyType==CONVEX_SHAPE_PROXYTYPE)
	  m_collisionAlgorithm= new btConvexConvexAlgorithm(NULL, ci,  m_collisionObject0, m_collisionObject1,simplexSolver, pdSolver,0,0);
}

BuoyantShapeConvexCollisionAlgorithm::~BuoyantShapeConvexCollisionAlgorithm()
{
}

void BuoyantShapeConvexCollisionAlgorithm::processCollision (btCollisionObject* body0,btCollisionObject* body1,const btDispatcherInfo& dispatchInfo,btManifoldResult* resultOut)
{

	btHfFluidBuoyantConvexShape* tmpShape0 = (btHfFluidBuoyantConvexShape*)m_collisionObject0->getCollisionShape();
	//btHfFluidBuoyantConvexShape* tmpShape1 = (btHfFluidBuoyantConvexShape*)body1->getCollisionShape();
	btConvexShape* convexShape0 = tmpShape0->getConvexShape();
	//btConvexShape* convexShape1 = tmpShape1->getConvexShape();


	m_collisionObject0->setCollisionShape (convexShape0);
	//body1->setCollisionShape (convexShape1);

	if(m_proxyType==COMPOUND_SHAPE_PROXYTYPE)
  	  m_collisionAlgorithm->processCollision (m_collisionObject0, m_collisionObject1, dispatchInfo,resultOut);
	else if(m_proxyType==TRIANGLE_MESH_SHAPE_PROXYTYPE)
  	  m_collisionAlgorithm->processCollision (m_collisionObject1, m_collisionObject0, dispatchInfo,resultOut);
	else if(m_proxyType==CONVEX_SHAPE_PROXYTYPE){
	  m_collisionAlgorithm->processCollision (m_collisionObject0, m_collisionObject1, dispatchInfo,resultOut);	
	}

	m_collisionObject0->setCollisionShape (tmpShape0);
	//body1->setCollisionShape (tmpShape1);

   
}

btScalar BuoyantShapeConvexCollisionAlgorithm::calculateTimeOfImpact(btCollisionObject* body0,btCollisionObject* body1,const btDispatcherInfo& dispatchInfo,btManifoldResult* resultOut)
{
	btHfFluidBuoyantConvexShape* tmpShape0 = (btHfFluidBuoyantConvexShape*)m_collisionObject0->getCollisionShape();
	//btHfFluidBuoyantConvexShape* tmpShape1 = (btHfFluidBuoyantConvexShape*)body1->getCollisionShape();
	btConvexShape* convexShape0 = tmpShape0->getConvexShape();
	//btConvexShape* convexShape1 = tmpShape1->getConvexShape();


	m_collisionObject0->setCollisionShape (convexShape0);
	//body1->setCollisionShape (convexShape1);

	btScalar toi = btScalar(0.0f);

	//toi = m_convexConvexAlgorithm.calculateTimeOfImpact (body0, body1, dispatchInfo, resultOut);

	if(m_proxyType==COMPOUND_SHAPE_PROXYTYPE)
  	  toi=m_collisionAlgorithm->calculateTimeOfImpact (m_collisionObject0, m_collisionObject1, dispatchInfo,resultOut);
	else if(m_proxyType==TRIANGLE_MESH_SHAPE_PROXYTYPE)
  	  toi=m_collisionAlgorithm->calculateTimeOfImpact (m_collisionObject1, m_collisionObject0, dispatchInfo,resultOut);
	else if(m_proxyType==CONVEX_SHAPE_PROXYTYPE)
	  toi=m_collisionAlgorithm->calculateTimeOfImpact (m_collisionObject0, m_collisionObject1, dispatchInfo,resultOut);

	m_collisionObject0->setCollisionShape (tmpShape0);
	//body1->setCollisionShape (tmpShape1);

	return toi;
}
