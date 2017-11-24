#include <dccomms_ros_msgs/AddAcousticChannel.h>
#include <dccomms_ros_msgs/CheckChannel.h>
#include <thread>
#include <uwsim/AcousticCommsChannel.h>

namespace uwsim {

AcousticCommsChannel::AcousticCommsChannel(AcousticCommsChannelConfig cfg) {
  config = cfg;
  _addChannelService = _nh.serviceClient<dccomms_ros_msgs::AddAcousticChannel>(
      "/dccomms_netsim/add_acoustic_channel");
  _checkChannelService = _nh.serviceClient<dccomms_ros_msgs::CheckChannel>(
      "/dccomms_netsim/check_channel");
  std::thread worker(&AcousticCommsChannel::_Work, this);
  worker.detach();
}

bool AcousticCommsChannel::_Check() {
  bool res = true;
  dccomms_ros_msgs::CheckChannel srv;

  srv.request.id = config.id;

  if (!_checkChannelService.call(srv)) {
    res = false;
  }

  return res && srv.response.exists;
}

void AcousticCommsChannel::_Work() {
  while (!_Check()) {
    _Add();
    std::this_thread::sleep_for(std::chrono::seconds(4));
  }
}

bool AcousticCommsChannel::_Add() {
  dccomms_ros_msgs::AddAcousticChannel srv;
  srv.request.id = config.id;

  if (!_addChannelService.call(srv)) {
    ROS_ERROR("fail adding channel '%d'", srv.request.id);
    return false;
  } else {
    ROS_INFO("AcousticCommsChannel: '%d' added", srv.request.id);
    return true;
  }
}
}
