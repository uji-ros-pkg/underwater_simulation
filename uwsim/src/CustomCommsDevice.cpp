#include <osg/Node>
#include <osg/PositionAttitudeTransform>
#include <pluginlib/class_list_macros.h>
#include <thread>
#include <uwsim/CustomCommsDevice.h>
#include <uwsim/SimulatedIAUV.h>
#include <dccomms_ros_msgs/types.h>
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

bool CustomCommsDevice::_Add() {
  dccomms_ros_msgs::AddCustomDevice srv;

  srv.request.frameId = this->config->tfId;
  srv.request.dccommsId = this->config->dccommsId;
  srv.request.mac = this->config->mac;
  srv.request.maxDistance = this->config->maxDistance;
  srv.request.minDistance = this->config->minDistance;
  srv.request.minPktErrorRate = this->config->minPktErrRatio;
  srv.request.pktErrorRateIncPerMeter = this->config->pktErrRatioIncPerMeter;
  srv.request.bitrate = this->config->bitrate;
  srv.request.bitrateSd = this->config->bitrateSd;
  srv.request.maxTxFifoSize = this->config->txFifoSize;

  ROS_INFO("CustomCommsDevice  ID = %s ; Frame = %s",
           srv.request.dccommsId.c_str(), srv.request.frameId.c_str());
  if (!_addService.call(srv)) {
    ROS_ERROR("fail adding '%s'", srv.request.dccommsId.c_str());
    return false;
  } else {
    ROS_INFO("CustomCommsDevice '%s' added", srv.request.dccommsId.c_str());
  }

  // link dev to tx channel
  dccomms_ros_msgs::LinkDeviceToChannel ldchSrv;
  ldchSrv.request.dccommsId = this->config->dccommsId;
  ldchSrv.request.channelId = this->config->txChannelId;
  ldchSrv.request.linkType = dccomms_ros::CHANNEL_LINK_TYPE::CHANNEL_TX;
  if (!_linkToChannelService.call(ldchSrv)) {
    ROS_ERROR("fail linking dev to tx channel");
    return false;
  } else {
    ROS_INFO("comms dev linked to tx channel");
    return true;
  }

  // link dev to rx channel
  ldchSrv.request.dccommsId = this->config->dccommsId;
  ldchSrv.request.channelId = this->config->rxChannelId;
  ldchSrv.request.linkType = dccomms_ros::CHANNEL_LINK_TYPE::CHANNEL_RX;
  if (!_linkToChannelService.call(ldchSrv)) {
    ROS_ERROR("fail linking dev to rx channel");
    return false;
  } else {
    ROS_INFO("comms dev linked to rx channel");
    return true;
  }
}
void CustomCommsDevice::SetConfig(CommsDevice_Config *cfg) {
  config = dynamic_cast<CustomCommsDevice_Config *>(cfg);
}

CommsDevice_Config *CustomCommsDevice::GetConfig() { return config; }

CustomCommsDevice::CustomCommsDevice(CustomCommsDevice_Config *cfg,
                                     osg::ref_ptr<osg::Node> target,
                                     SimulatedIAUV *auv)
    : CommsDevice(cfg, target, auv) {
  Init(cfg, target, auv);
  _addService = this->node.serviceClient<dccomms_ros_msgs::AddCustomDevice>(
      "/dccomms_netsim/add_custom_net_device");
  _linkToChannelService =
      this->node.serviceClient<dccomms_ros_msgs::LinkDeviceToChannel>(
          "/dccomms_netsim/link_dev_to_channel");
}

CommsDevice *CustomCommsDevice_Factory::Create(CommsDevice_Config *cfg,
                                               osg::ref_ptr<osg::Node> target,
                                               SimulatedIAUV *auv) {
  CustomCommsDevice_Config *config =
      dynamic_cast<CustomCommsDevice_Config *>(cfg);
  return new CustomCommsDevice(config, target, auv);
}


PLUGINLIB_EXPORT_CLASS(CustomCommsDevice_Factory, uwsim::SimulatedDeviceFactory)
