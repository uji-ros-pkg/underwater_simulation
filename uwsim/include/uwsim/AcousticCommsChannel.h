#ifndef UWSIM_ACOUSTICCOMMSCHANNEL_
#define UWSIM_ACOUSTICCOMMSCHANNEL_

#include <ros/ros.h>
#include <uwsim/ConfigXMLParser.h>
#include <uwsim/NetSim.h>

namespace uwsim {

class AcousticCommsChannel {
public:
  AcousticCommsChannelConfig config;
  AcousticCommsChannel(AcousticCommsChannelConfig cfg);
private:
  bool _AddToNetSim();

};
}
#endif
