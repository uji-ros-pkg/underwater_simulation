#include <dccomms_ros_msgs/AddCustomChannel.h>
#include <dccomms_ros_msgs/CheckChannel.h>
#include <thread>
#include <uwsim/CustomCommsChannel.h>

namespace uwsim {

CustomCommsChannel::CustomCommsChannel(CustomCommsChannelConfig cfg) {
  config = cfg;
  _addChannelService = _nh.serviceClient<dccomms_ros_msgs::AddCustomChannel>(
      "/dccomms_netsim/add_custom_channel");
  _checkChannelService = _nh.serviceClient<dccomms_ros_msgs::CheckChannel>(
      "/dccomms_netsim/check_channel");
  std::thread worker(&CustomCommsChannel::_Work, this);
  worker.detach();
}

bool CustomCommsChannel::_Check() {
  bool res = true;
  dccomms_ros_msgs::CheckChannel srv;

  srv.request.id = config.id;

  if (!_checkChannelService.call(srv)) {
    res = false;
  }

  return res && srv.response.exists;
}

void CustomCommsChannel::_Work() {
  while (true) {
    if (!_Check()) {
        _Add();
        std::this_thread::sleep_for(std::chrono::seconds(4));
    }
  }
}

bool CustomCommsChannel::_Add() {
  dccomms_ros_msgs::AddCustomChannel srv;
  srv.request.id = config.id;
  srv.request.minPrTime = config.minPropTime;
  srv.request.prTimeIncPerMeter = config.propTimeIncPerMeter;

  if (!_addChannelService.call(srv)) {
    ROS_ERROR("fail adding channel '%d'", srv.request.id);
    return false;
  } else {
    ROS_INFO("CustomCommsChannel: '%d' added", srv.request.id);
    return true;
  }
}

}
