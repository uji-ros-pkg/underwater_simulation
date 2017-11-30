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

LedArray::LedArray(osg::ref_ptr<osg::Group> root) {
  sceneRoot = root;
  float ledRadio = 0.02;
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
  sceneRoot->getOrCreateStateSet()->setMode(numLight, osg::StateAttribute::ON);

  numLedLights++;

  greenLightNum = numLedLights;
  numLight = GL_LIGHT0 + numLedLights % 8; // maximum: 8 lights!
  greenLightSource = LightBuilder::createLightSource(
      numLedLights, osg::Vec4(0.0f, 1.0f, 0.0f, 1.0f), att);
  greenLightSource->getLight()->setLinearAttenuation(0.1);
  greenLightSource->getLight()->setQuadraticAttenuation(0.1);
  numLight = GL_LIGHT0 + numLedLights % 8; // maximum: 8 lights!
  greenLightNum = numLight;
  sceneRoot->getOrCreateStateSet()->setMode(numLight, osg::StateAttribute::ON);

  // Add to scene graph
  vMRedLight = (osg::Transform *)new osg::PositionAttitudeTransform();
  vMGreenLight = (osg::Transform *)new osg::PositionAttitudeTransform();

  vMRedLight->asPositionAttitudeTransform()->setPosition(osg::Vec3d(0, 0.1, 0));
  vMGreenLight->asPositionAttitudeTransform()->setPosition(
      osg::Vec3d(0, -0.1, 0));

  vMRedLight->addChild(redLightGeode);
  vMRedLight->addChild(redLightSource.get());

  vMGreenLight->addChild(greenLightGeode);
  vMGreenLight->addChild(greenLightSource.get());

  node = new osg::Group();
  node->asGroup()->addChild(vMRedLight.get());
  node->asGroup()->addChild(vMGreenLight.get());
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
    break;
  case GREEN_LED:
    num = greenLightNum;
    material = greenLightMaterial;
    color = osg::Vec4(0.0, 1.0, 0.0, 1.0);
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

void LedArray::StartAnimationTest() {
  std::thread redLightWorker([this]() {
    while (1) {
      std::this_thread::sleep_for(chrono::milliseconds(700));
      UpdateLetState(RED_LED, true);
      UpdateLetState(GREEN_LED, false);
      std::this_thread::sleep_for(chrono::milliseconds(700));
      UpdateLetState(RED_LED, false);
      UpdateLetState(GREEN_LED, true);
    }
  });
  redLightWorker.detach();
}

osg::ref_ptr<osg::Node> LedArray::GetOSGNode() { return node.get(); }
}
// osg::ref_ptr<osg::Transform> vMl =
//    (osg::Transform *)new osg::PositionAttitudeTransform;
// vMl->asPositionAttitudeTransform()->setPosition(osg::Vec3d(
//    cfg->position[0], cfg->position[1], cfg->position[2]));
// auv->urdf->link[target]
//    ->getParent(0)
//    ->getParent(0)
//    ->asGroup()
//    ->addChild(vMl);
// dev->commsLeds =
//    std::shared_ptr<CommsLights>(new CommsLights(auv->root));
// vMl->addChild(dev->commsLeds->GetOSGNode().get());
// dev->commsLeds->StartAnimationTest();
