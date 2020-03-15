#include <uwsim/NED.h>

namespace uwsim {

GeographicLib::LocalCartesian NED::localCartesian;
GeographicLib::Geocentric NED::earth;
std::mutex NED::localCartesian_mutex;

void NED::SetOrigin(double lat, double lon, double alt) {
  localCartesian_mutex.lock();
  earth = GeographicLib::Geocentric(GeographicLib::Constants::WGS84_a(),
                                    GeographicLib::Constants::WGS84_f());
  localCartesian = GeographicLib::LocalCartesian(lat, lon, alt, earth);
  localCartesian_mutex.unlock();
}

void NED::GetNED(double lat, double lon, double alt, double &x, double &y,
                 double &z) {
  localCartesian_mutex.lock();
  //https://github.com/mavlink/mavros/issues/216
  double enux, enuy, enuz;
  localCartesian.Forward(lat, lon, alt, enux, enuy, enuz);
  x = enuy;
  y = enux;
  z = -enuz;
  localCartesian_mutex.unlock();
}
}
