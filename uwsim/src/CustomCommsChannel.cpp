#include <uwsim/CustomCommsChannel.h>
#include <dccomms_ros_msgs/AddCustomChannel.h>
#include <thread>

namespace uwsim {

CustomCommsChannel::CustomCommsChannel(CustomCommsChannelConfig cfg) {
  config = cfg;
  std::thread worker(&CustomCommsChannel::_Work, this);
  worker.detach();
}
void CustomCommsChannel::_Work()
{

}
}
