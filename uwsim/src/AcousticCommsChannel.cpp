#include <dccomms_ros_msgs/AddAcousticChannel.h>
#include <dccomms_ros_msgs/CheckChannel.h>
#include <thread>
#include <uwsim/AcousticCommsChannel.h>

namespace uwsim {

AcousticCommsChannel::AcousticCommsChannel(AcousticCommsChannelConfig cfg) {
  config = cfg;
  _AddToNetSim();
}

bool AcousticCommsChannel::_AddToNetSim() {
  dccomms_ros_msgs::AddAcousticChannelRequest srv;
  srv.id = config.id;

  auto netsim = NetSim::GetSim();
  netsim->AddAcousticChannel(srv);
}
}
