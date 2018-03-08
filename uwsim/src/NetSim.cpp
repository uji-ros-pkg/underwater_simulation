#ifndef UWSIM_NETSIM_H
#define UWSIM_NETSIM_H

#include <class_loader/multi_library_class_loader.h>
#include <uwsim/NetSim.h>
namespace uwsim {

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

NetSimTracing::NetSimTracing() {}

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
      [](std::string path, ROSCommsDevicePtr dev, PacketPtr pkt) {
        tracing->Info("{}: (ID: {} ; MAC: {}) Transmitting packet", path,
                      dev->GetDccommsId());
      };

  ROSCommsDevice::PacketReceivedCallback rxcb = [](
      std::string path, ROSCommsDevicePtr dev, PacketPtr pkt) {
    tracing->Info("{}: (ID: {} ; MAC: {}) Packet received from {} ({} bytes)",
                  path, dev->GetDccommsId(), dev->GetMac(), pkt->GetSrcAddr(),
                  pkt->GetPacketSize());
  };

  ns3::Config::Connect("/ROSDeviceList/*/PacketTransmitting",
                       ns3::MakeCallback(txcb));

  ns3::Config::Connect("/ROSDeviceList/*/PacketReceived",
                       ns3::MakeCallback(rxcb));
}
}
#endif
