#include <dccomms_ros_msgs/types.h>
#include <osg/Node>
#include <osg/PositionAttitudeTransform>
#include <pluginlib/class_list_macros.h>
#include <thread>
#include <uwsim/CustomCommsDevice.h>
#include <uwsim/SimulatedIAUV.h>
/* You will need to add your code HERE */
#include <tf/transform_broadcaster.h>
#include <thread>
#include <uwsim/UWSimUtils.h>

uint32_t CustomCommsDevice::nDevsReady = 0;
uint32_t CustomCommsDevice::nDevs = 0;

SimulatedDeviceConfig::Ptr
CustomCommsDevice_Factory::processConfig(const xmlpp::Node *node,
                                         ConfigFile *config) {
  CustomCommsDevice_Config *cfg = new CustomCommsDevice_Config(getType());
  processCommonConfig(node, config, cfg);
  xmlpp::Node::NodeList list = node->get_children();

  cfg->intrinsicDelay = 0;
  cfg->bitrate = 1000;
  cfg->bitrateSd = 0;
  cfg->maxDistance = 99999999;
  cfg->minDistance = 0;
  cfg->pktErrRatioIncPerMeter = 0;
  cfg->minPktErrRatio = 0;

  for (xmlpp::Node::NodeList::iterator iter = list.begin(); iter != list.end();
       ++iter) {

    const xmlpp::Node *child = dynamic_cast<const xmlpp::Node *>(*iter);
    if (child->get_name() == "intrinsicDelay")
      config->extractUIntChar(child, cfg->intrinsicDelay);
    else if (child->get_name() == "bitrate")
      config->extractFloatChar(child, cfg->bitrate);
    else if (child->get_name() == "bitrateSd")
      config->extractFloatChar(child, cfg->bitrateSd);
    else if (child->get_name() == "maxDistance")
      config->extractUIntChar(child, cfg->maxDistance);
    else if (child->get_name() == "minDistance")
      config->extractUIntChar(child, cfg->minDistance);
    else if (child->get_name() == "minPktErrRatio")
      config->extractFloatChar(child, cfg->minPktErrRatio);
    else if (child->get_name() == "pktErrRatioIncPerMeter")
      config->extractFloatChar(child, cfg->pktErrRatioIncPerMeter);
    else if (child->get_name() == "txChannelId")
      config->extractUIntChar(child, cfg->txChannelId);
    else if (child->get_name() == "rxChannelId")
      config->extractUIntChar(child, cfg->rxChannelId);
  }
  return SimulatedDeviceConfig::Ptr(cfg);
}

bool CustomCommsDevice::_AddToNetSim() {
  auto netsim = NetSim::GetSim();

  dccomms_ros_msgs::AddCustomDeviceRequest srv;

  srv.frameId = this->config->tfId;
  srv.dccommsId = this->config->dccommsId;
  srv.mac = this->config->mac;
  srv.maxDistance = this->config->maxDistance;
  srv.minDistance = this->config->minDistance;
  srv.minPktErrorRate = this->config->minPktErrRatio;
  srv.pktErrorRateIncPerMeter = this->config->pktErrRatioIncPerMeter;
  srv.bitrate = this->config->bitrate;
  srv.bitrateSd = this->config->bitrateSd;
  srv.maxTxFifoSize = this->config->txFifoSize;

  ROS_INFO("CustomCommsDevice  ID = %s ; Frame = %s",
           srv.dccommsId.c_str(), srv.frameId.c_str());

  netsim->AddCustomDevice(srv);

  ROS_INFO("CustomCommsDevice '%s' added", srv.dccommsId.c_str());

  // link dev to tx channel
  dccomms_ros_msgs::LinkDeviceToChannelRequest ldchSrv;
  ldchSrv.dccommsId = this->config->dccommsId;
  ldchSrv.channelId = this->config->txChannelId;
  ldchSrv.linkType = dccomms_ros::CHANNEL_LINK_TYPE::CHANNEL_TX;

  netsim->LinkDevToChannel(ldchSrv);

  ROS_INFO("comms dev linked to tx channel");

  // link dev to rx channel
  ldchSrv.dccommsId = this->config->dccommsId;
  ldchSrv.channelId = this->config->rxChannelId;
  ldchSrv.linkType = dccomms_ros::CHANNEL_LINK_TYPE::CHANNEL_RX;

  netsim->LinkDevToChannel(ldchSrv);

  return true;
}
void CustomCommsDevice::SetConfig(CommsDevice_Config *cfg) {
  config = dynamic_cast<CustomCommsDevice_Config *>(cfg);
}

CommsDevice_Config *CustomCommsDevice::GetConfig() { return config; }

CustomCommsDevice::CustomCommsDevice(CustomCommsDevice_Config *cfg,
                                     osg::ref_ptr<osg::Node> target,
                                     SimulatedIAUV *auv)
    : UWSimCommsDevice(cfg, target, auv) {
  Init(cfg, target, auv);
}

UWSimCommsDevice *CustomCommsDevice_Factory::Create(CommsDevice_Config *cfg,
                                               osg::ref_ptr<osg::Node> target,
                                               SimulatedIAUV *auv) {
  CustomCommsDevice_Config *config =
      dynamic_cast<CustomCommsDevice_Config *>(cfg);
  return new CustomCommsDevice(config, target, auv);
}

PLUGINLIB_EXPORT_CLASS(CustomCommsDevice_Factory, uwsim::SimulatedDeviceFactory)
