#ifndef UWSIM_NETSIM_
#define UWSIM_NETSIM_

#include <dccomms/DataLinkFrame.h>
#include <dccomms_ros/simulator/ROSCommsSimulator.h>

namespace uwsim {
using namespace dccomms_ros;

class NetSimTracing : public cpplogging::Logger {
public:
  NetSimTracing();
  virtual void Configure();
};
class NetSim {
public:
  static ns3::Ptr<ROSCommsSimulator> GetSim();
  static void LoadDefaultTracingScript();
  static void LoadTracingScript(const std::string &className,
                                const std::string &libPath);

};

}

#endif
