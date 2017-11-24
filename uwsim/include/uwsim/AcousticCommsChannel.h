#ifndef UWSIM_ACOUSTICCOMMSCHANNEL_
#define UWSIM_ACOUSTICCOMMSCHANNEL_

#include <ros/ros.h>
#include <uwsim/ConfigXMLParser.h>

namespace uwsim {

class AcousticCommsChannel {
public:
  AcousticCommsChannelConfig config;
  AcousticCommsChannel(AcousticCommsChannelConfig cfg);
private:
  ros::NodeHandle _nh;
  ros::ServiceClient _checkChannelService, _addChannelService;
  void _Work();
  bool _Check();
  bool _Add();

};
}
#endif
