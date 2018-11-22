#include <uwsim/NetSim.h>
#include <dccomms_ros/simulator/NetsimLogFormatter.h>

namespace uwsim {

NetSimTracingPtr NetSim::_script;
std::shared_ptr<class_loader::ClassLoader> NetSim::_loader;

ns3::Ptr<ROSCommsSimulator> NetSim::GetSim() {
  static ns3::Ptr<ROSCommsSimulator> ptr = 0;
  if (ptr == 0) {
    ptr = ns3::CreateObject<ROSCommsSimulator>();
    auto pb = dccomms::CreateObject<dccomms::DataLinkFramePacketBuilder>(
        dccomms::DataLinkFrame::crc16);
    ptr->SetDefaultPacketBuilder(pb);
  }
  return ptr;
}

void NetSim::LoadTracingScript(const std::string &className,
                               const std::string &libPath) {
  _loader = std::shared_ptr<class_loader::ClassLoader>(new class_loader::ClassLoader(libPath));
  std::vector<std::string> classes =
      _loader->getAvailableClasses<NetSimTracing>();
  for (unsigned int c = 0; c < classes.size(); ++c) {
    if (classes[c] == className) {
      NetSimTracing *tr =
          _loader->createUnmanagedInstance<NetSimTracing>(classes[c]);
      _script = std::shared_ptr<NetSimTracing>(tr);
      _script->Configure();
      break;
    }
  }
}

void NetSim::LoadDefaultTracingScript() {
  _script = NetSimTracingPtr(new NetSimTracing());
  _script->Configure();
}

NetSimTracingPtr NetSim::GetScript() { return _script; }

NetSimTracing::NetSimTracing() {
  SetLogFormatter(std::make_shared<NetsimLogFormatter>("%v"));
  LogToConsole(false);
}

void NetSimTracing::Configure() {
  SetLogName("uwsim_netsim");

  /*
  In order to use an object created outside the lambda function
  we have to make it global or capture the object. However, capturing
  does not work if the lambda function is going to be converted to a
  function pointer. So we make all this objects static.
  From the std
  (https://stackoverflow.com/questions/28746744/passing-lambda-as-function-pointer):
  "The closure type for a lambda-expression with no lambda-capture
   has a public non-virtual non-explicit const conversion function to
   pointer to function having the same parameter and return types as the
   closure type’s function call operator. The value returned by this
   conversion function shall be the address of a function that, when invoked,
   has the same effect as invoking the closure type’s function call operator."
  */

  static NetSimTracing *tracing = this;
  ROSCommsDevice::PacketTransmittingCallback txcb =
      [](std::string path, ROSCommsDevicePtr dev, ns3PacketPtr pkt) {
//        tracing->Info("{}: (ID: {} ; MAC: {}) Transmitting packet", path,
//                      dev->GetDccommsId(), dev->GetMac());
      };

  ROSCommsDevice::PacketReceivedCallback rxcb = [](
      std::string path, ROSCommsDevicePtr dev, ns3PacketPtr pkt) {
//    NetsimHeader header;
//    pkt->PeekHeader(header);
//    tracing->Info("{}: (ID: {} ; MAC: {}) Packet received from {} ({} bytes)",
//                  path, dev->GetDccommsId(), dev->GetMac(), header.GetSrc(),
//                  header.GetPacketSize());
  };

  ns3::Config::Connect("/ROSDeviceList/*/PacketTransmitting",
                       ns3::MakeCallback(txcb));

  ns3::Config::Connect("/ROSDeviceList/*/PacketReceived",
                       ns3::MakeCallback(rxcb));
}

void NetSimTracing::Run() { DoRun(); }
void NetSimTracing::DoRun() {}
}
