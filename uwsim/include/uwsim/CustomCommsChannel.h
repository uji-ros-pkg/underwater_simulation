#ifndef UWSIM_CUSTOMCOMMSCHANNEL_
#define UWSIM_CUSTOMCOMMSCHANNEL_

#include <ros/ros.h>
#include <uwsim/ConfigXMLParser.h>
#include <uwsim/NetSim.h>

namespace uwsim {

class CustomCommsChannel {
public:
  CustomCommsChannelConfig config;
  CustomCommsChannel(CustomCommsChannelConfig cfg);
private:
  bool _AddToNetSim();

};
}
#endif
