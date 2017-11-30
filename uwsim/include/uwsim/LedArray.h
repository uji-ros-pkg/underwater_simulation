#ifndef UWSIM_LEDARRAY_
#define UWSIM_LEDARRAY_

#include <chrono>
#include <osg/Geometry>
#include <osg/LightSource>
#include <osg/Material>
#include <ros/ros.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

namespace uwsim {

class LedArray {
public:
  enum ledType { RED_LED, GREEN_LED };
  LedArray(osg::ref_ptr<osg::Group> sceneRoot);
  osg::ref_ptr<osg::Node> GetOSGNode();
  void StartAnimationTest();
  void UpdateLetState(ledType, bool);

private:
  osg::ref_ptr<osg::Group> sceneRoot;
  osg::ref_ptr<osg::Transform> vMRedLight;
  osg::ref_ptr<osg::Transform> vMGreenLight;
  osg::ref_ptr<osg::Node> node;
  static uint32_t numLedLights;
  osg::ref_ptr<osg::LightSource> greenLightSource, redLightSource;
  osg::ref_ptr<osg::Material> greenLightMaterial, redLightMaterial;
  uint32_t redLightNum, greenLightNum;
};
}
#endif
