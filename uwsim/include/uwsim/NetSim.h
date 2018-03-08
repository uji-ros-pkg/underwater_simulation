#ifndef UWSIM_NETSIM_
#define UWSIM_NETSIM_

#include <dccomms_ros/simulator/ROSCommsSimulator.h>
#include <dccomms/DataLinkFrame.h>

namespace uwsim {
using namespace dccomms_ros;
class NetSim {
public:
    static ns3::Ptr<ROSCommsSimulator> GetSim();

};

class NetSimTracing : public cpplogging::Logger{
public:
  NetSimTracing();
  virtual void Configure();
};
}

#endif
