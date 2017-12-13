//! Header file of the profiling sonar sensor code.
/*!
 * Made by Olaya Alvarez.
 * Modification from a file by
 * Mario Prats and Javier Perez
 *
 * This file contains the classes which define the
 * Profiling Sonar sensor, including the parser,
 * the device itself, and the ROS Interfaces.
 */

#ifndef MULTIBEAMSENSOR_ECHO_H_
#define MULTIBEAMSENSOR_ECHO_H_

#include "VirtualCamera.h"

//For the barebone
#include "SimulatedDevices.h"
#include <ros/ros.h>

//For the parser
#include "ConfigXMLParser.h"

//Creating the device
#include <osg/Node>
#include "SimulatedIAUV.h"
#include <osg/PositionAttitudeTransform>

//ROS Interfaces
#include "ROSInterface.h"
#include <tf/transform_broadcaster.h>

using namespace uwsim;


//! Parser class of the sensor.
/*! This parser class receives the data from the XML an creates the logic structure that allows to build the sensor in the simulator.
 *  It holds the data from the XML and is the input for the device class. */
class ProfilingSonar_Config : public SimulatedDeviceConfig
{
public:
  //XML members
  std::string relativeTo;               /*!< Link from which we position the sensor */
  std::string linkName;                 /*!< Link name */
  double position[3], orientation[3];   /*!< Relative pose */
  double range;         /*!< Maximun distance from the that can be detected */
  double initAngle;     /*!< Initial angle from where the sensor starts to read */
  double finalAngle;    /*!< Final angle where the sensor ends the reading */
  double angleIncr;     /*!< Angle increment (in degrees) between consecutive measurements */
  double fov;           /*!< Field Of View (in degrees) of the camera. */
  double alpha;         /*!< Angle increment (in degrees) */
  int visible;          /*!< when set to 1, it shows the Profiling Sonar beams on visualizer. When set to 0, it doesn't */

  //! Constructor
  ProfilingSonar_Config(std::string type_) :
      SimulatedDeviceConfig(type_)
  {
  }
};


//! Driver/ROSInterface factory class
class ProfilingSonar_Factory : public SimulatedDeviceFactory
{
public:
  //! Profiling Sonar factory Constructor.
  /*! This is the only place the device/interface type is set */
  ProfilingSonar_Factory(std::string type_ = "ProfilingSonar") :
      SimulatedDeviceFactory(type_)
  {
  }

  //! Method from Factory class to create the barebone
  /*! This method iterates through a node with the XML information for the sensor, and extract it with the functions
   * provided by the parser include. Once this information is extracted, the new node is ready to create a new device.
   * \param node receives a node from libXML++ with the Profiling Sonar sensor on it
   * \param config provides methods to extract the XML data from \param node.
   */
  SimulatedDeviceConfig::Ptr processConfig(const xmlpp::Node* node,
                                           ConfigFile * config);

  //! This method moves from the config object to the device object.
  /*! This method is called iteratively until every device is configured or it reaches maximum iterations.
   * It iterates over all the simulated devices, and if it is ProfilingSonar, casts the config class, creates
   * the device and adds it to the device factory.
   *  \param auv where the device (and other simulated devices) is attached.
   *  \param vehicleChars the vehicle configuration
   *  \param SceneBuilder unused, since this sensor has no representation
   *  \param iteration variable to iterate over all the simulated devices
   */
  bool applyConfig(SimulatedIAUV * auv, Vehicle &vehicleChars, SceneBuilder *sceneBuilder, size_t iteration);

  //! This method creates the link between the ProfilingSonar sensor and the publisher
  std::vector<boost::shared_ptr<ROSInterface> > getInterface(ROSInterfaceInfo & rosInterface,
                                                             std::vector<boost::shared_ptr<SimulatedIAUV> > & iauvFile);
};


//! Class of the Profiling Sonar device itself
/*! The Profiling Sonar class inherits from SimulatedDevice, like all the sensors in UWSim that
 * are created from the plugin */
class ProfilingSonar : public SimulatedDevice
{
  struct Remap
  {
    int pixel1, pixel2;
    double weight1, weight2;
    double distort;
  };

public:
  osg::Node *parent;                    /*!< Parent OSG node of the sensor */
  std::string relativeTo;               /*!< Link from which we position the sensor */
  double position[3], orientation[3];   /*!< Variables which define the Pose of the sensor */
  std::vector<VirtualCamera> vcams;     /*!< Array with the Virtual Cameras */
  std::string name;                     /*!< Name of the device */
  std::string parentLinkName;           /*!< Name of the parent link */
  int numpixels;                        /*!< Number of pixels (in total) */
  int camPixels;                        /*!< Pixels of each camera */
  int nCams;                            /*!< Number of cameras necessary to profile the surface */
  double range;                         /*!< Maximun distance from the that can be detected */
  double initAngle;                     /*!< Initial angle from where the sensor starts to read */
  double finalAngle;                    /*!< Final angle where the sensor ends the reading */
  double angleIncr;                     /*!< Angle increment (in degrees) between consecutive measurements */
  double camsFOV;                       /*!< Field of View of each camera */
  osg::ref_ptr<osg::Geode> geode;       /*!< Geometry node that draws the beam */
  std::vector<Remap> remapVector;       /*!< Applies distortion to the measurement received */
  osg::Node *trackNode;                 /*!< OSG node with the pose information (which is already available as son) */

  //! ProfilingSonar constructor
  /*! Here  father's constructor cfg is called to
   *  relate the sensor class with the config class. Then, the position convention is
   *  transformed from OSG convention to TF convention. */
  ProfilingSonar(ProfilingSonar_Config * cfg, osg::Node *trackNode);

  //! Function where all the sensing work and representation is done. Receives arguments from the XML.
  /*! This method calls the preCalcTable function. */
  void ProfilingSonarSensing(osg::Group *uwsim_root,
                             std::string name, std::string parentName,
                             osg::Node *trackNode, double initAngle,
                             double finalAngle, double alpha,
                             double range, double fov,
                             unsigned int mask, int visible,
                             unsigned int ARMask);
  //! Function that configure the cameras position and characteristics.
  void preCalcTable();

  //! Function which obains the pose of the sensor relative to the parent link.
  int getTFTransform(tf::Pose & pose, std::string & parent);

};

//! ROS interface configuration to publish all the data extracted
/*! This class inheritts from ROSPublisherInterface in order to
 * create its own thread to publish the data and be independent from
 * simulation steps. */
class ProfilingSonar_ROSPublisher : public ROSPublisherInterface
{
    ProfilingSonar * dev;

public:

  //! Constructor.
    /*! It copies a pointer to the device and calls ROSPublisherInterface
     * constructor with topic name and rate.*/
  ProfilingSonar_ROSPublisher(ProfilingSonar *dev, std::string topic,
                              int rate) :
      ROSPublisherInterface(topic, rate), dev(dev)
  {
  }

  //! Creates the topic Publisher
  void createPublisher(ros::NodeHandle &nh);

  //! Publishes the topic with the position and LaserScan data
  void publish();

  ~ProfilingSonar_ROSPublisher()
  {
  }
};

#endif



