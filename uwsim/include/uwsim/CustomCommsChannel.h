#ifndef UWSIM_CUSTOMCOMMSCHANNEL_
#define UWSIM_CUSTOMCOMMSCHANNEL_

#include <ros/ros.h>
#include <uwsim/ConfigXMLParser.h>

namespace uwsim {

class CustomCommsChannel {
public:
  CustomCommsChannelConfig config;
  CustomCommsChannel(CustomCommsChannelConfig cfg);
private:
  ros::NodeHandle _nh;
  ros::ServiceClient _checkChannel, _addChannel;
  void _Work();

};
}
#endif
