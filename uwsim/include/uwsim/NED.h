#ifndef UWSIM_NED_
#define UWSIM_NED_

#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/Geoid.hpp>
#include <GeographicLib/LocalCartesian.hpp>
#include <cpplogging/cpplogging.h>

namespace uwsim {

class NED : public cpplogging::Logger{
public:
  static void SetOrigin(double lat, double lon, double alt);
  static void GetNED(double lat, double lon, double alt, double &x, double &y, double &z);

private:
  static GeographicLib::LocalCartesian localCartesian;
  static GeographicLib::Geocentric earth;
  static std::mutex localCartesian_mutex;

};

}

#endif
