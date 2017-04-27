#ifndef CommsDevice_ECHO_H_
#define CommsDevice_ECHO_H_
#include "SimulatedDevice.h"
#include <ros/ros.h>
#include "ConfigXMLParser.h"
#include "ROSInterface.h"
#include <dccomms_ros/AddDevice.h>

using namespace uwsim;

class CommsDevice_Config : public SimulatedDeviceConfig
{
public:
  //XML members
    std::string relativeTo, tfId, name;
    unsigned int devClass, mac;
    unsigned int maxBitRate, intrinsicDelay;
    double prTimeIncPerMeter, trTime, trTimeSd;
    unsigned int maxDistance, minDistance;
    double pktErrRatioIncPerMeter, minPktErrRatio;
    double position[3] {0,0,0}, orientation[3]{0,0,0};

  //constructor
  CommsDevice_Config(std::string type_) :
      SimulatedDeviceConfig(type_)
  {
  }
};

class CommsDevice : public SimulatedDevice
{
public:
  osg::Node *parent;
  SimulatedIAUV * auv;
  osg::ref_ptr<osg::Node> target;
  ros::NodeHandle node;
  CommsDevice_Config * config;

  CommsDevice(CommsDevice_Config * cfg, osg::ref_ptr<osg::Node> target, SimulatedIAUV * auv);
  void Start()
  {
    ros::ServiceClient client = node.serviceClient<dccomms_ros::AddDevice>("/dccomms_netsim/add_net_device");
    dccomms_ros::AddDevice srv;

    srv.request.frameId = this->config->tfId;
    srv.request.iddev = this->config->name;
    srv.request.mac = this->config->mac;
    srv.request.maxBitRate = this->config->maxBitRate;
    srv.request.maxDistance = this->config->maxDistance;
    srv.request.minDistance = this->config->minDistance;
    srv.request.minPktErrorRate = this->config->minPktErrRatio;
    srv.request.pktErrorRateIncPerMeter = this->config->pktErrRatioIncPerMeter;
    srv.request.prTimeIncPerMeter = this->config->prTimeIncPerMeter;
    srv.request.trTimeMean = this->config->trTime;
    srv.request.trTimeSd = this->config->trTimeSd;
    srv.request.devType = this->config->devClass;

    ROS_INFO("FrameId = %s", srv.request.frameId.c_str());
    if(client.call(srv))
    {
        ROS_INFO("CommsDevice Added");
    }
    else
    {
        ROS_ERROR("Failed to add CommsDevice");
    }

  }
};

/* You will need to add your code HERE */
class CommsDevice_Factory : public SimulatedDeviceFactory
{
public:
  //this is the only place the device/interface type is set
  CommsDevice_Factory(std::string type_ = "CommsDevice") :
      SimulatedDeviceFactory(type_)
  {
  }
  ;

  SimulatedDeviceConfig::Ptr processConfig(const xmlpp::Node* node, ConfigFile * config);
  bool applyConfig(SimulatedIAUV * auv, Vehicle &vehicleChars, SceneBuilder *sceneBuilder, size_t iteration);
  std::vector<boost::shared_ptr<ROSInterface> > getInterface(ROSInterfaceInfo & rosInterface,
                               std::vector<boost::shared_ptr<SimulatedIAUV> > & iauvFile);
};

class CommsDevice_ROSPublisher : public ROSPublisherInterface
{
  CommsDevice * dev;
public:
  CommsDevice_ROSPublisher(CommsDevice *dev, std::string topic, int rate) :
      ROSPublisherInterface(topic, rate), dev(dev)
  {
  }

  void createPublisher(ros::NodeHandle &nh);
  void publish();

  ~CommsDevice_ROSPublisher()
  {
  }
};

#endif
