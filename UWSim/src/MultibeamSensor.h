#ifndef MULTIBEAMSENSOR_H_
#define MULTIBEAMSENSOR_H_

#include "VirtualCamera.h"


class MultibeamSensor: public VirtualCamera{
  struct Remap{
    int pixel1,pixel2;
    double weight1,weight2;
    double distort;
  };

  public:
    int numpixels;
    double range,initAngle,finalAngle,angleIncr;
    std::vector<Remap> remapVector;
    MultibeamSensor(osg::Group *uwsim_root, std::string name, osg::Node *trackNode, double initAngle,double finalAngle,double alpha,double range);
    void preCalcTable();
};

#endif
