/*
 BuoyantShape vs ConvexSHape collision algorithm added to Experimental Buoyancy fluid demo.
 */

#ifndef BUOYANTSHAPECONVEXCOLLISIONALGORITHM_H
#define BUOYANTSHAPECONVEXCOLLISIONALGORITHM_H

#include "BulletCollision/BroadphaseCollision/btCollisionAlgorithm.h"
#include "BulletCollision/BroadphaseCollision/btBroadphaseProxy.h"
#include "BulletCollision/CollisionDispatch/btCollisionCreateFunc.h"
#include "BulletCollision/NarrowPhaseCollision/btPersistentManifold.h"
#include "BulletCollision/CollisionDispatch/btCollisionDispatcher.h"
#include "BulletCollision/CollisionShapes/btTriangleCallback.h"
#include "BulletCollision/CollisionDispatch/btConvexConvexAlgorithm.h"
#include "BulletCollision/CollisionDispatch/btCompoundCollisionAlgorithm.h"

#include "LinearMath/btVector3.h"
class btHfFluid;

class btConvexConvexAlgorithm;
class btConvexPenetrationDepthSolver;
class btSimplexSolverInterface;

///experimental buyancy fluid demo
/// btHfFluidBuoyantShapeCollisionAlgorithm  provides collision detection between btHfFluidBuoyantConvexShape and btConvexShape
class BuoyantShapeConvexCollisionAlgorithm : public btCollisionAlgorithm
{
  btCollisionObject* m_collisionObject0;
  btCollisionObject* m_collisionObject1;

  ///for convex vs buoyant (instead of buoyant vs convex), we use this swapped boolean
  bool m_isSwapped;
  int m_proxyType;

  btCollisionAlgorithm * m_collisionAlgorithm;
public:

  BuoyantShapeConvexCollisionAlgorithm(const btCollisionAlgorithmConstructionInfo& ci, btCollisionObject* col0,
                                       btCollisionObject* col1, btSimplexSolverInterface* simplexSolver,
                                       btConvexPenetrationDepthSolver* pdSolver, bool isSwapped, int proxyType);

  virtual ~BuoyantShapeConvexCollisionAlgorithm();

  virtual void processCollision(btCollisionObject* body0, btCollisionObject* body1,
                                const btDispatcherInfo& dispatchInfo, btManifoldResult* resultOut);

  virtual btScalar calculateTimeOfImpact(btCollisionObject* body0, btCollisionObject* body1,
                                         const btDispatcherInfo& dispatchInfo, btManifoldResult* resultOut);

  virtual void getAllContactManifolds(btManifoldArray& manifoldArray)
  {
    m_collisionAlgorithm->getAllContactManifolds(manifoldArray);
  }

  struct CreateFunc : public btCollisionAlgorithmCreateFunc
  {
    btConvexPenetrationDepthSolver* m_pdSolver;
    btSimplexSolverInterface* m_simplexSolver;
    int m_proxyType;

    CreateFunc(btSimplexSolverInterface* simplexSolver, btConvexPenetrationDepthSolver* pdSolver, int proxyType)
    {
      m_simplexSolver = simplexSolver;
      m_pdSolver = pdSolver;
      m_proxyType = proxyType;
    }

    virtual ~CreateFunc()
    {
    }
    virtual btCollisionAlgorithm* CreateCollisionAlgorithm(btCollisionAlgorithmConstructionInfo& ci,
                                                           btCollisionObject* body0, btCollisionObject* body1)
    {
      void* mem = ci.m_dispatcher1->allocateCollisionAlgorithm(sizeof(BuoyantShapeConvexCollisionAlgorithm));
      if (!m_swapped)
      {
        return new (mem) BuoyantShapeConvexCollisionAlgorithm(ci, body0, body1, m_simplexSolver, m_pdSolver, false,
                                                              m_proxyType);
      }
      else
      {
        return new (mem) BuoyantShapeConvexCollisionAlgorithm(ci, body0, body1, m_simplexSolver, m_pdSolver, true,
                                                              m_proxyType);
      }
    }
  };
};

#endif //BUOYANTSHAPECONVEXCOLLISIONALGORITHM_H
