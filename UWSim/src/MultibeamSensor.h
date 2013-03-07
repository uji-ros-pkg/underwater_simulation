#ifndef MULTIBEAMSENSOR_H_
#define MULTIBEAMSENSOR_H_

#include "VirtualCamera.h"


class MultibeamSensor: public VirtualCamera{

  public:
    int numpixels;
    MultibeamSensor(osg::Group *uwsim_root, std::string name, osg::Node *trackNode, int width,double fov):VirtualCamera(uwsim_root,name,trackNode,width,fov){this->numpixels=width;};

};

#endif
