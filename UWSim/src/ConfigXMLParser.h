#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

#ifndef CONFIGXMLPARSER_H_
#define CONFIGXMLPARSER_H_

#include <libxml++/libxml++.h>
#include <urdf/model.h>

#include <iostream>
using namespace std;
#include <cstdlib>
#include <list>

struct ROSInterfaceInfo{
  typedef enum {ROSOdomToPAT, PATToROSOdom, ROSJointStateToArm, ArmToROSJointState, VirtualCameraToROSImage, ROSImageToHUD} type_t;
  string topic, infoTopic, targetName;
  type_t type; //Type of ROSInterface, 0: ROSOdomToPAT ,1:PATToROSOdom, 2: ROSJointStateToArm, 3: ArmToROSJointState, 4:VirtualCameraToROSImage
  int rate; //if it's necessary
  unsigned int w, h; //width and height if necessary
  unsigned int posx, posy; ///< default (x,y) position of the widget if necessary
  double scale; ///< default scale of the widget if necessary
};

struct Parameters{
  double fx,fy,x0,y0,f,n,k;
};

struct Vcam{
  string name;
  string linkName, roscam, roscaminfo;
  int resw,resh,link;
  double position[3],orientation[3];
  Parameters * parameters;
  void init(){name="";linkName="";roscam="";roscaminfo="";resw=160;resh=120;position[0]=0;position[1]=0;position[2]=0;orientation[0]=0;orientation[1]=0;orientation[2]=0;parameters=NULL;}
};

struct Mimic{
  string jointName;
  double offset,multiplier;
};

struct Link{
  string name;
  string file; // only used in mesh type
  int type; //Related to geometry, 0: mesh from file, 1:box, 2:cylinder, 3:sphere
  double position[3];
  double rpy[3];
  double quat[4];
  double boxSize[3]; //only used in box type
  double length, radius; //only used in cylinder and sphere types
  int material;
};

struct Joint{
  string name;
  int parent, child; //references to Link
  int mimicp,type; //0 fixed, 1 rotation, 2 prismatic.
  float lowLimit,upLimit;
  Mimic *mimic;
  double position[3];
  double rpy[3];
  double axis[3];
  double quat[4];
};

struct Material{
  string name;
  double r,g,b,a;
};

struct Vehicle{
  string name;
  Link * links;
  Joint * joints;
  int nlinks;
  int njoints;
  int ninitJoints;
  int nmaterials;
  double position[3];
  double orientation[3];
  double * jointValues;
  Material * materials;
  list <Vcam> Vcams;
};

struct Object{
  string name,file;
  double position[3];
  double orientation[3];
  double offsetp[3];
  double offsetr[3];
};

class ConfigFile{
private:

  void esPi(string in,double * param);

  void extractFloatChar(const xmlpp::Node* node,double * param);
  void extractIntChar(const xmlpp::Node* node,int * param);
  void extractUIntChar(const xmlpp::Node* node, unsigned int * param);
  void extractStringChar(const xmlpp::Node* node,string * param);
  void extractPositionOrColor(const xmlpp::Node* node,double * param);
  void extractOrientation(const xmlpp::Node* node,double * param);

  void proccessFog(const xmlpp::Node* node);
  void proccessOceanState(const xmlpp::Node* node);
  void proccessSimParams(const xmlpp::Node* node);
  void proccessParameters(const xmlpp::Node*, Parameters * params);
  void proccessVcam(const xmlpp::Node* node, Vcam * vcam);
  void proccessCamera(const xmlpp::Node* node);
  void proccessJointValues(const xmlpp::Node* node,double ** jointValues,int * ninitJoints);
  void proccessVehicle(const xmlpp::Node* node,Vehicle *vehicle);
  void proccessObject(const xmlpp::Node* node,Object *object);
  void proccessROSInterface(const xmlpp::Node* node,ROSInterfaceInfo * rosInterface);
  void proccessROSInterfaces(const xmlpp::Node* node);
  void proccessXML(const xmlpp::Node* node);


  void proccessPose(urdf::Pose pose,double * position, double * rpy,double * quat);
  int proccessVisual(boost::shared_ptr<const urdf::Visual> visual,Link * link,int nmat,Material * materials); //returns current material
  void proccessJoint(boost::shared_ptr<const urdf::Joint> joint,Joint * jointVehicle,int parentLink,int childLink);
  int proccessLink(boost::shared_ptr<const urdf::Link> link,Vehicle * vehicle,int nlink,int njoint,int nmat,Material * materials); //returns current link number
  int proccessURDFFile(string file, Vehicle * vehicle);

  void postProccessVehicle(Vehicle * vehicle);

public:
  double windx, windy,windSpeed,depth, reflectionDamping, waveScale, choppyFactor, crestFoamHeight, oceanSurfaceHeight,fogDensity;
  int isNotChoppy, disableShaders, eye_in_hand,freeMotion,resw,resh ;
  string arm, vehicleToTrack;
  double camPosition[3],camLookAt[3],fogColor[3],color[3],attenuation[3], offsetr[3], offsetp[3];
  double camFov, camAspectRatio, camNear, camFar;
  list <Vehicle> vehicles;
  list <Object> objects;
  list <ROSInterfaceInfo> ROSInterfaces;

  ConfigFile(const std::string &fName);
};

#endif
