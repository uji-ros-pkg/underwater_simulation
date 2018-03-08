#include <dccomms_ros_msgs/AddCustomChannel.h>
#include <dccomms_ros_msgs/CheckChannel.h>
#include <thread>
#include <uwsim/CustomCommsChannel.h>

namespace uwsim {

CustomCommsChannel::CustomCommsChannel(CustomCommsChannelConfig cfg) {
  config = cfg;
  _AddToNetSim();
}

bool CustomCommsChannel::_AddToNetSim() {
  dccomms_ros_msgs::AddCustomChannelRequest srv;
  srv.id = config.id;
  srv.minPrTime = config.minPropTime;
  srv.prTimeIncPerMeter = config.propTimeIncPerMeter;

  auto netsim = NetSim::GetSim();
  netsim->AddCustomChannel(srv);
}
}
