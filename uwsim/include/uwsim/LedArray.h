#ifndef UWSIM_LEDARRAY_
#define UWSIM_LEDARRAY_

#include <chrono>
//#include <condition_variable>
//#include <mutex>
#include <osg/Geometry>
#include <osg/LightSource>
#include <osg/Material>
#include <ros/ros.h>
#include <ros/subscriber.h>
#include <tf/transform_broadcaster.h>
#include <thread>
#include <underwater_sensor_msgs/LedLight.h>
#include <uwsim/ConfigXMLParser.h>

namespace uwsim {

class LedArray {
public:
  enum ledType { RED_LED, GREEN_LED };
  LedArray(osg::ref_ptr<osg::Group> sceneRoot, LedArrayConfig config);
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
  ros::Subscriber redLedSubscriber;
  ros::Subscriber greenLedSubscriber;
  std::thread ledWorker, testWorker;
  LedArrayConfig config;

  struct Action {
    ros::Time offTime;
  };

  //std::mutex actionsMutex;
  //std::condition_variable actionsCond;
  bool redStateOn, greenStateOn;
  Action redAction, greenAction;

  void InitROSInterface();
  void HandleNewLedState(underwater_sensor_msgs::LedLightConstPtr msg, ledType);
  void CheckAndUpdateLed(const ros::Time & now, ledType type, bool , const Action &);
};
}
#endif
