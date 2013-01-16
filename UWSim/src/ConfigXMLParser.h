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
  typedef enum {Unknown, ROSOdomToPAT, PATToROSOdom, ROSJointStateToArm, ArmToROSJointState, VirtualCameraToROSImage, RangeSensorToROSRange,ROSImageToHUD, ROSTwistToPAT, ROSPoseToPAT, ImuToROSImu, PressureSensorToROS, GPSSensorToROS, DVLSensorToROS, RangeImageSensorToROSImage} type_t;
  string topic, infoTopic, targetName;
  type_t type; //Type of ROSInterface
  int rate; //if it's necessary
  unsigned int w, h; //width and height if necessary
  unsigned int posx, posy, blackWhite; ///< default (x,y) position of the widget if necessary, blackWhite camera
  double scale; ///< default scale of the widget if necessary
  int visualize; ///< If 1, enable visualization of the data. 0 by default
  double color[3]; // visualization color in rosodomtopat waypoints
};

struct Parameters{
  double fx,fy,x0,y0,f,n,k;
};

struct Vcam{
  string name;
  string linkName, roscam, roscaminfo;
  std::string frameId; ///Frame Id for stereo camera images
  int resw,resh,link,range;
  double showpath;
  double position[3],orientation[3];
  double baseLine; ///baseline for stereo cameras
  boost::shared_ptr<Parameters> parameters;
  void init(){name="";linkName="";roscam="";roscaminfo="";resw=160;resh=120;position[0]=0;position[1]=0;position[2]=0;orientation[0]=0;orientation[1]=0;orientation[2]=0; baseLine=0.0; frameId=""; showpath=0; parameters.reset();range=0;}
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

struct Imu {
  string name;
  string linkName;
  double std; //standard deviation
  double position[3],orientation[3];
  int link;
  void init(){name="";linkName=""; std=0.0; position[0]=0;position[1]=0;position[2]=0;orientation[0]=0;orientation[1]=0;orientation[2]=0;}
};

struct XMLPressureSensor {
  string name;
  string linkName;
  double std; //standard deviation
  double position[3],orientation[3];
  int link;
  void init(){name="";linkName=""; std=0.0; position[0]=0;position[1]=0;position[2]=0;orientation[0]=0;orientation[1]=0;orientation[2]=0;}
};

struct XMLGPSSensor {
  string name;
  string linkName;
  double std; //standard deviation
  double position[3],orientation[3];
  int link;
  void init(){name="";linkName=""; std=0.0; position[0]=0;position[1]=0;position[2]=0;orientation[0]=0;orientation[1]=0;orientation[2]=0;}
};

struct XMLDVLSensor {
  string name;
  string linkName;
  double std; //standard deviation
  double position[3],orientation[3];
  int link;
  void init(){name="";linkName=""; std=0.0; position[0]=0;position[1]=0;position[2]=0;orientation[0]=0;orientation[1]=0;orientation[2]=0;}
};

struct Mimic{
  string jointName;
  double offset,multiplier;
};

struct Geometry{
  int type; //Related to geometry, 0: mesh from file, 1:box, 2:cylinder, 3:sphere, 4:NoVisual
  double boxSize[3]; //only used in box type
  double length, radius; //only used in cylinder and sphere types
  string file; // only used in mesh type
};

struct Link{
  string name;
  double position[3];
  double rpy[3];
  double quat[4];
  int material;
  boost::shared_ptr<Geometry> cs, geom;
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
  std::list<Vcam> VRangecams;
  std::list<rangeSensor> range_sensors, object_pickers;
  std::list<Imu> imus;
  std::list<XMLPressureSensor> pressure_sensors;
  std::list<XMLGPSSensor> gps_sensors;
  std::list<XMLDVLSensor> dvl_sensors;
};

struct PhysicProperties{
  double mass;
  double inertia[3];
  std::string csType;
  void init(){mass=1;inertia[0]=0;inertia[1]=0;inertia[2]=0;csType="box";};
};

struct Object{
  string name,file;
  double position[3];
  double orientation[3];
  double offsetp[3];
  double offsetr[3];
  boost::shared_ptr<PhysicProperties> physicProperties;
};

struct PhysicsWater{
  int enable;
  double position[3];
  double resolution;
  double size[6];
  void init(){enable=0;resolution=0.25;position[0]=position[1]=position[2]=0;size[0]=size[2]=size[4]=-10;size[1]=size[3]=size[5]=10;};
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
  void processPhysicsWater(const xmlpp::Node* node);
  void processSize(const xmlpp::Node* node);
  void processParameters(const xmlpp::Node*, Parameters *params);
  void processVcam(const xmlpp::Node* node, Vcam &vcam);
  void processRangeSensor(const xmlpp::Node* node, rangeSensor &rs);
  void processImu(const xmlpp::Node* node, Imu &rs);
  void processPressureSensor(const xmlpp::Node* node, XMLPressureSensor &ps);
  void processDVLSensor(const xmlpp::Node* node, XMLDVLSensor &s);
  void processGPSSensor(const xmlpp::Node* node, XMLGPSSensor &s);
  void processCamera(const xmlpp::Node* node);
  void processJointValues(const xmlpp::Node* node, std::vector<double> &jointValues, int &ninitJoints);
  void processVehicle(const xmlpp::Node* node, Vehicle &vehicle);
  void processPhysicProperties(const xmlpp::Node* node, PhysicProperties &pp);
  void processObject(const xmlpp::Node* node, Object &object);
  void processROSInterface(const xmlpp::Node* node, ROSInterfaceInfo &rosInterface);
  void processROSInterfaces(const xmlpp::Node* node);
  void processXML(const xmlpp::Node* node);

  void processGeometry(urdf::Geometry * geometry, Geometry * geom);
  void processPose(urdf::Pose pose,double position[3], double rpy[3],double quat[4]);
  int processVisual(boost::shared_ptr<const urdf::Visual> visual, Link &link, int nmat, std::vector<Material> &materials); //returns current material
  void processJoint(boost::shared_ptr<const urdf::Joint> joint, Joint &jointVehicle,int parentLink,int childLink);
  int processLink(boost::shared_ptr<const urdf::Link> link, Vehicle &vehicle, int nlink, int njoint, int nmat, std::vector<Material> &materials); //returns current link number
  int processURDFFile(string file, Vehicle &vehicle);

  void postprocessVehicle(Vehicle &vehicle);

public:
  double windx, windy,windSpeed,depth, reflectionDamping, waveScale, choppyFactor, crestFoamHeight, oceanSurfaceHeight,fogDensity;
  int isNotChoppy, disableShaders, eye_in_hand,freeMotion,resw,resh,enablePhysics ;
  string arm, vehicleToTrack;
  double camPosition[3],camLookAt[3],fogColor[3],color[3],attenuation[3], offsetr[3], offsetp[3],gravity[3];
  double camFov, camAspectRatio, camNear, camFar;
  list <Vehicle> vehicles;
  list <Object> objects;
  list <ROSInterfaceInfo> ROSInterfaces;
  PhysicsWater physicsWater;

  ConfigFile(const std::string &fName);
};

#endif
