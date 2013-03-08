#ifndef MULTIBEAMSENSOR_H_
#define MULTIBEAMSENSOR_H_

#include "VirtualCamera.h"


class MultibeamSensor: public VirtualCamera{
  struct Remap{
    int pixel1,pixel2;
    double weight1,weight2;
  };

  public:
    int numpixels;
    std::vector<Remap> remapVector;
    MultibeamSensor(osg::Group *uwsim_root, std::string name, osg::Node *trackNode, int width,double fov);
    void preCalcTable(double fov);
};

#endif
