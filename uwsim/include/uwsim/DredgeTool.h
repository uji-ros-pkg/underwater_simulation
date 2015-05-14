///////////////////////////////////////////////////////////////////////////////////////////////////////////////
//"Echo" example, SimulatedDevice_Echo.h
#ifndef DREDGE_TOOL_H_
#define DREDGE_TOOL_H_
#include "SimulatedDevice.h"
using namespace uwsim;

/*
 * Example header of driver/rosinterface configuration/factory
 *
 * Included in SimulatedDevices.cpp
 */

//Driver/ROSInterface configuration
class DredgeTool_Config : public SimulatedDeviceConfig
{
public:
  //XML members
  std::string target;
  double offsetp[3];
  double offsetr[3];
  //constructor
  DredgeTool_Config(std::string type_) :
      SimulatedDeviceConfig(type_)
  {
  }
};

//Driver/ROSInterface factory class
class DredgeTool_Factory : public SimulatedDeviceFactory
{
public:
  //this is the only place the device/interface type is set
  DredgeTool_Factory(std::string type_ = "DredgeTool") :
      SimulatedDeviceFactory(type_)
  {
  }
  ;

  SimulatedDeviceConfig::Ptr processConfig(const xmlpp::Node* node, ConfigFile * config);
  bool applyConfig(SimulatedIAUV * auv, Vehicle &vehicleChars, SceneBuilder *sceneBuilder, size_t iteration);
};

//can be a sparate header file for actual implementation classes...

#include "ConfigXMLParser.h"
#include "ROSInterface.h"
#include <ros/ros.h>
#include "UWSimUtils.h"

//Driver class
class DredgeTool : public SimulatedDevice, public AbstractDredgeTool
{
  osg::ref_ptr<osg::Node> target;

  void applyPhysics(BulletPhysics * bulletPhysics)
  {
  }
public:
  std::string info; //Device's property

  DredgeTool(DredgeTool_Config * cfg, osg::ref_ptr<osg::Node> target);

  virtual boost::shared_ptr<osg::Matrix> getDredgePosition();

  void dredgedParticles(int nparticles);


};


#endif
