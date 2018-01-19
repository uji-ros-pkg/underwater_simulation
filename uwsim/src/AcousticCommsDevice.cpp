#include <osg/Node>
#include <osg/PositionAttitudeTransform>
#include <pluginlib/class_list_macros.h>
#include <thread>
#include <uwsim/AcousticCommsDevice.h>
#include <uwsim/SimulatedIAUV.h>

/* You will need to add your code HERE */
#include <tf/transform_broadcaster.h>
#include <thread>
#include <uwsim/UWSimUtils.h>

uint32_t AcousticCommsDevice::nDevsReady = 0;
uint32_t AcousticCommsDevice::nDevs = 0;

SimulatedDeviceConfig::Ptr
AcousticCommsDevice_Factory::processConfig(const xmlpp::Node *node,
                                         ConfigFile *config) {
  AcousticCommsDevice_Config *cfg = new AcousticCommsDevice_Config(getType());
  processCommonConfig(node, config, cfg);
  xmlpp::Node::NodeList list = node->get_children();

  for (xmlpp::Node::NodeList::iterator iter = list.begin(); iter != list.end();
       ++iter) {

    const xmlpp::Node *child = dynamic_cast<const xmlpp::Node *>(*iter);
//    if (child->get_name() == "maxBitRate")
//      config->extractUIntChar(child, cfg->maxBitRate);
//    else if (child->get_name() == "intrinsicDelay")
//      config->extractUIntChar(child, cfg->intrinsicDelay);
//    else if (child->get_name() == "bitrate")
//      config->extractFloatChar(child, cfg->bitrate);
//    else if (child->get_name() == "bitrateSd")
//      config->extractFloatChar(child, cfg->bitrateSd);
//    else if (child->get_name() == "maxDistance")
//      config->extractUIntChar(child, cfg->maxDistance);
//    else if (child->get_name() == "minDistance")
//      config->extractUIntChar(child, cfg->minDistance);
//    else if (child->get_name() == "minPktErrRatio")
//      config->extractFloatChar(child, cfg->minPktErrRatio);
  }
  return SimulatedDeviceConfig::Ptr(cfg);
}

bool AcousticCommsDevice::_Add() {
  dccomms_ros_msgs::AddAcousticDevice srv;

  srv.request.frameId = this->config->tfId;
  srv.request.dccommsId = this->config->dccommsId;
  srv.request.mac = this->config->mac;
  srv.request.maxTxFifoSize = this->config->txFifoSize;

  ROS_INFO("AcousticCommsDevice  ID = %s ; Frame = %s",
           srv.request.dccommsId.c_str(), srv.request.frameId.c_str());
  if (!_addService.call(srv)) {
    ROS_ERROR("fail adding '%s'", srv.request.dccommsId.c_str());
    return false;
  } else {
    ROS_INFO("AcousticCommsDevice '%s' added", srv.request.dccommsId.c_str());
  }

  // link dev to channel
  dccomms_ros_msgs::LinkDeviceToChannel ldchSrv;
  ldchSrv.request.dccommsId = this->config->dccommsId;
  ldchSrv.request.channelId = this->config->channelId;
  if (!_linkToChannelService.call(ldchSrv)) {
    ROS_ERROR("fail linking dev to channel");
    return false;
  } else {
    ROS_INFO("comms dev linked to channel");
    return true;
  }
}
void AcousticCommsDevice::SetConfig(CommsDevice_Config *cfg) {
  config = dynamic_cast<AcousticCommsDevice_Config *>(cfg);
}

CommsDevice_Config *AcousticCommsDevice::GetConfig() { return config; }

AcousticCommsDevice::AcousticCommsDevice(AcousticCommsDevice_Config *cfg,
                                     osg::ref_ptr<osg::Node> target,
                                     SimulatedIAUV *auv)
    : CommsDevice(cfg, target, auv) {
  Init(cfg, target, auv);
  _addService = this->node.serviceClient<dccomms_ros_msgs::AddAcousticDevice>(
      "/dccomms_netsim/add_acoustic_net_device");
  _linkToChannelService =
      this->node.serviceClient<dccomms_ros_msgs::LinkDeviceToChannel>(
          "/dccomms_netsim/link_dev_to_channel");
}

CommsDevice *AcousticCommsDevice_Factory::Create(CommsDevice_Config *cfg,
                                               osg::ref_ptr<osg::Node> target,
                                               SimulatedIAUV *auv) {
  AcousticCommsDevice_Config *config =
      dynamic_cast<AcousticCommsDevice_Config *>(cfg);
  return new AcousticCommsDevice(config, target, auv);
}


PLUGINLIB_EXPORT_CLASS(AcousticCommsDevice_Factory, uwsim::SimulatedDeviceFactory)
