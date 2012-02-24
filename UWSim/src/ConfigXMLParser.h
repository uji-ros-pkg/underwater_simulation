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
  typedef enum {Unknown, ROSOdomToPAT, PATToROSOdom, ROSJointStateToArm, ArmToROSJointState, VirtualCameraToROSImage, RangeSensorToROSRange, ROSImageToHUD, ROSTwistToPAT} type_t;
  string topic, infoTopic, targetName;
  type_t type; //Type of ROSInterface
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
  boost::shared_ptr<Parameters> parameters;
  void init(){name="";linkName="";roscam="";roscaminfo="";resw=160;resh=120;position[0]=0;position[1]=0;position[2]=0;orientation[0]=0;orientation[1]=0;orientation[2]=0; parameters.reset();}
};

struct rangeSensor {
  string name;
  string linkName;
  double position[3],orientation[3];
  double range;
  int visible;
  int link;
  void init(){name="";linkName="";position[0]=0;position[1]=0;position[2]=0;orientation[0]=0;orientation[1]=0;orientation[2]=0;range=0;visible=0;}
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
  boost::shared_ptr<Mimic> mimic;
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
  std::vector<Link> links;
  std::vector<Joint> joints;
  int nlinks;
  int njoints;
  int ninitJoints;
  int nmaterials;
  double position[3];
  double orientation[3];
  std::vector<double> jointValues;
  std::vector<Material> materials;
  std::list<Vcam> Vcams;
  std::list<rangeSensor> range_sensors;
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

  void esPi(string in,double &param);

  void extractFloatChar(const xmlpp::Node* node,double &param);
  void extractIntChar(const xmlpp::Node* node,int &param);
  void extractUIntChar(const xmlpp::Node* node, unsigned int &param);
  void extractStringChar(const xmlpp::Node* node,string &param);
  void extractPositionOrColor(const xmlpp::Node* node,double param[3]);
  void extractOrientation(const xmlpp::Node* node,double param[3]);

  void processFog(const xmlpp::Node* node);
  void processOceanState(const xmlpp::Node* node);
  void processSimParams(const xmlpp::Node* node);
  void processParameters(const xmlpp::Node*, Parameters *params);
  void processVcam(const xmlpp::Node* node, Vcam &vcam);
  void processRangeSensor(const xmlpp::Node* node, rangeSensor &rs);
  void processCamera(const xmlpp::Node* node);
  void processJointValues(const xmlpp::Node* node, std::vector<double> &jointValues, int &ninitJoints);
  void processVehicle(const xmlpp::Node* node, Vehicle &vehicle);
  void processObject(const xmlpp::Node* node, Object &object);
  void processROSInterface(const xmlpp::Node* node, ROSInterfaceInfo &rosInterface);
  void processROSInterfaces(const xmlpp::Node* node);
  void processXML(const xmlpp::Node* node);


  void processPose(urdf::Pose pose,double position[3], double rpy[3],double quat[4]);
  int processVisual(boost::shared_ptr<const urdf::Visual> visual, Link &link, int nmat, std::vector<Material> &materials); //returns current material
  void processJoint(boost::shared_ptr<const urdf::Joint> joint, Joint &jointVehicle,int parentLink,int childLink);
  int processLink(boost::shared_ptr<const urdf::Link> link, Vehicle &vehicle, int nlink, int njoint, int nmat, std::vector<Material> &materials); //returns current link number
  int processURDFFile(string file, Vehicle &vehicle);

  void postprocessVehicle(Vehicle &vehicle);

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
