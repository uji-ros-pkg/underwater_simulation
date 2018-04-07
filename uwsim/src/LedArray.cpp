#include <osg/Geode>
#include <osg/Node>
#include <osg/PositionAttitudeTransform>
#include <osg/ShapeDrawable>
#include <thread>
#include <uwsim/LedArray.h>
#include <uwsim/UWSimUtils.h>

using namespace std;
using namespace osg;

namespace uwsim {
uint32_t LedArray::numLedLights = 0;

LedArray::LedArray(osg::ref_ptr<osg::Group> root, LedArrayConfig config)
    : nh(config.name) {
  sceneRoot = root;
  double ledRadio = config.radio;
  double offset = config.space / 2;
  osg::Geode *redLightGeode = new osg::Geode();
  redLightGeode->addDrawable(
      new osg::ShapeDrawable(new osg::Sphere(osg::Vec3(), ledRadio)));
  redLightMaterial = new osg::Material();
  redLightMaterial->setDiffuse(osg::Material::FRONT,
                               osg::Vec4(1.0, 0.0, 0.0, 1.0));
  redLightMaterial->setAmbient(osg::Material::FRONT,
                               osg::Vec4(1.0, 0.0, 0.0, 1.0));

  osg::StateSet *redSphereStateSet = redLightGeode->getOrCreateStateSet();
  redSphereStateSet->ref();
  redSphereStateSet->setAttribute(redLightMaterial);

  osg::Geode *greenLightGeode = new osg::Geode();
  greenLightGeode->addDrawable(
      new osg::ShapeDrawable(new osg::Sphere(osg::Vec3(), ledRadio)));
  greenLightMaterial = new osg::Material();
  greenLightMaterial->setDiffuse(osg::Material::FRONT,
                                 osg::Vec4(0.0, 1.0, 0.0, 1.0));
  greenLightMaterial->setAmbient(osg::Material::FRONT,
                                 osg::Vec4(0.0, 1.0, 0.0, 1.0));
  osg::StateSet *greenSphereStateSet = greenLightGeode->getOrCreateStateSet();
  greenSphereStateSet->ref();
  greenSphereStateSet->setAttribute(greenLightMaterial);

  numLedLights++;

  float att = 1;

  redLightSource = LightBuilder::createLightSource(
      numLedLights, osg::Vec4(1.0f, 0.0f, 0.0f, 1.0f), att);
  redLightSource->getLight()->setLinearAttenuation(0.1);
  redLightSource->getLight()->setQuadraticAttenuation(0.1);
  uint32_t numLight = GL_LIGHT0 + numLedLights % 8; // maximum: 8 lights!
  redLightNum = numLight;
  sceneRoot->getOrCreateStateSet()->setMode(numLight, osg::StateAttribute::OFF);

  numLedLights++;

  greenLightNum = numLedLights;
  numLight = GL_LIGHT0 + numLedLights % 8; // maximum: 8 lights!
  greenLightSource = LightBuilder::createLightSource(
      numLedLights, osg::Vec4(0.0f, 1.0f, 0.0f, 1.0f), att);
  greenLightSource->getLight()->setLinearAttenuation(0.1);
  greenLightSource->getLight()->setQuadraticAttenuation(0.1);
  numLight = GL_LIGHT0 + numLedLights % 8; // maximum: 8 lights!
  greenLightNum = numLight;
  sceneRoot->getOrCreateStateSet()->setMode(numLight, osg::StateAttribute::OFF);

  // Add to scene graph
  vMRedLight = (osg::Transform *)new osg::PositionAttitudeTransform();
  vMGreenLight = (osg::Transform *)new osg::PositionAttitudeTransform();

  vMRedLight->asPositionAttitudeTransform()->setPosition(
      osg::Vec3d(0, offset, 0));
  vMGreenLight->asPositionAttitudeTransform()->setPosition(
      osg::Vec3d(0, -offset, 0));

  vMRedLight->addChild(redLightGeode);
  vMRedLight->addChild(redLightSource.get());

  vMGreenLight->addChild(greenLightGeode);
  vMGreenLight->addChild(greenLightSource.get());

  node = new osg::Group();
  node->asGroup()->addChild(vMRedLight.get());
  node->asGroup()->addChild(vMGreenLight.get());

  redStateOn = false;
  greenStateOn = false;

  this->config = config;
  InitROSInterface();
}

void LedArray::UpdateLetState(ledType type, bool on) {
  uint32_t num;
  osg::Material *material;
  osg::Vec4 color;
  switch (type) {
  case RED_LED:
    num = redLightNum;
    material = redLightMaterial;
    color = osg::Vec4(1.0, 0.0, 0.0, 1.0);
    redStateOn = on;
    break;
  case GREEN_LED:
    num = greenLightNum;
    material = greenLightMaterial;
    color = osg::Vec4(0.0, 1.0, 0.0, 1.0);
    greenStateOn = on;
    break;
  }
  if (on) {
    material->setEmission(osg::Material::FRONT, color);
    sceneRoot->getOrCreateStateSet()->setMode(num, osg::StateAttribute::ON);
  } else {
    material->setEmission(osg::Material::FRONT, osg::Vec4(0.0, 0.0, 0.0, 0.0));
    sceneRoot->getOrCreateStateSet()->setMode(num, osg::StateAttribute::OFF);
  }
}

void LedArray::HandleNewLedState(underwater_sensor_msgs::LedLightConstPtr msg,
                                 ledType type) {
  ros::Time offTime = ros::Time::now() + msg->duration;
  // actionsMutex.lock();
  switch (type) {
  case RED_LED:
    redAction.offTime = offTime;
    break;
  case GREEN_LED:
    greenAction.offTime = offTime;
    break;
  }
  // actionsCond.notify_one();
  // actionsMutex.unlock();
}

void LedArray::CheckAndUpdateLed(const ros::Time &now, ledType type, bool on,
                                 const Action &action) {
  if (on) {
    if (action.offTime <= now) {
      UpdateLetState(type, false);
    }
  } else if (action.offTime > now) {
    UpdateLetState(type, true);
  }
}
void LedArray::InitROSInterface() {
  redLedSubscriber = nh.subscribe<underwater_sensor_msgs::LedLight>(
      "red", 1,
      boost::bind(&LedArray::HandleNewLedState, this, _1, ledType::RED_LED));
  greenLedSubscriber = nh.subscribe<underwater_sensor_msgs::LedLight>(
      "green", 1,
      boost::bind(&LedArray::HandleNewLedState, this, _1, ledType::GREEN_LED));

  std::thread rosWorker([this]() {
    ros::Rate rate(20);
    while (1) {
      ros::Time now = ros::Time::now();
      CheckAndUpdateLed(now, RED_LED, redStateOn, redAction);
      CheckAndUpdateLed(now, GREEN_LED, greenStateOn, greenAction);
      rate.sleep();
    }
  });
  rosWorker.detach();
}

void LedArray::StartAnimationTest() {
  testWorker = std::thread([this]() {
    while (1) {
      std::this_thread::sleep_for(chrono::milliseconds(700));
      UpdateLetState(RED_LED, true);
      UpdateLetState(GREEN_LED, false);
      std::this_thread::sleep_for(chrono::milliseconds(700));
      UpdateLetState(RED_LED, false);
      UpdateLetState(GREEN_LED, true);
    }
  });
  testWorker.detach();
}

osg::ref_ptr<osg::Node> LedArray::GetOSGNode() { return node.get(); }
}
