#ifndef CustomCommsDevice_ECHO_H_
#define CustomCommsDevice_ECHO_H_
#include "ConfigXMLParser.h"
#include "ROSInterface.h"
#include "SimulatedDevice.h"
#include <dccomms_ros_msgs/AddCustomChannel.h>
#include <dccomms_ros_msgs/AddCustomDevice.h>
#include <dccomms_ros_msgs/CheckDevice.h>
#include <dccomms_ros_msgs/LinkDeviceToChannel.h>
#include <dccomms_ros_msgs/RemoveDevice.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <uwsim/CommsDevice.h>

using namespace uwsim;

class CustomCommsDevice_Config : public CommsDevice_Config {

public:
  // XML members
  unsigned int intrinsicDelay;
  double bitrate, bitrateSd;
  unsigned int maxDistance, minDistance;
  double pktErrRatioIncPerMeter, minPktErrRatio;

  // constructor
  CustomCommsDevice_Config(std::string type_) : CommsDevice_Config(type_) {}
};

class CustomCommsDevice : public CommsDevice {
public:
  CustomCommsDevice_Config *config;
  CustomCommsDevice(CustomCommsDevice_Config *cfg,
                    osg::ref_ptr<osg::Node> target, SimulatedIAUV *auv);
  void Start();
  static uint32_t nDevsReady;
  static uint32_t nDevs;
  void SetConfig(CommsDevice_Config *cfg);
  CommsDevice_Config *GetConfig();

protected:
  bool _Add();

private:
  ros::ServiceClient _addService, _linkToChannelService;
};

/* You will need to add your code HERE */
class CustomCommsDevice_Factory : public CommsDevice_Factory {
public:
  // this is the only place the device/interface type is set
  CustomCommsDevice_Factory(std::string type_ = "CustomCommsDevice")
      : CommsDevice_Factory(type_){};
  CommsDevice *Create(CommsDevice_Config *cfg, osg::ref_ptr<osg::Node> target,
                      SimulatedIAUV *auv);
  SimulatedDeviceConfig::Ptr processConfig(const xmlpp::Node *node,
                                           ConfigFile *config);
};

class CustomCommsDevice_ROSPublisher : public CommsDevice_ROSPublisher {
public:
  CustomCommsDevice_ROSPublisher(CustomCommsDevice *dev, std::string topic,
                                 int rate)
      : CommsDevice_ROSPublisher(dev, topic, rate) {}

  ~CustomCommsDevice_ROSPublisher() {}
};

#endif
