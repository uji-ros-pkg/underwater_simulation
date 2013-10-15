/*
 * Copyright (c) 2013 Tallinn University of Technology.
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the GNU Public License v3.0
 * which accompanies this distribution, and is available at
 * http://www.gnu.org/licenses/gpl.html
 *
 * Contributors:
 *     Yuri Gavshin
 */

#ifndef SIMULATEDDEVICE_H_
#define SIMULATEDDEVICE_H_

#include <libxml++/libxml++.h>
#include <iostream>
#include <cstdlib>
#include <list>
#include <boost/smart_ptr/shared_ptr.hpp>

struct ROSInterfaceInfo;
struct SimulatedIAUV;
struct ROSInterface;
struct ConfigFile;
struct Vehicle;
struct SceneBuilder;

namespace uwsim {
  struct SimulatedDevice;
  //Base class for device's XML configuration
  class SimulatedDeviceConfig{
    //device/rosinterface type identifier for both "XML config" and a "factory"
    std::string type;
    
  public:
    typedef boost::shared_ptr<SimulatedDeviceConfig> Ptr;
    //common XML properties:
    std::string name;
    std::string getType(){return type;}
    
    SimulatedDeviceConfig(std::string type);
    virtual ~SimulatedDeviceConfig(){};
  };

  //Base class for device/rosinterface "factory" 
  class SimulatedDeviceFactory{
    //device/rosinterface type identifier for both "XML config" and a "factory"
    std::string type;
  public:
    typedef boost::shared_ptr<SimulatedDeviceFactory> Ptr;
    std::string getType(){return type;}
    SimulatedDeviceFactory(std::string type);

    //DRIVER: parses XML and returns "XML config", executed first
    virtual SimulatedDeviceConfig::Ptr processConfig(const xmlpp::Node* node, ConfigFile * config) = 0;
    //DRIVER: checks parsed XML configurations and sets SimulatedAUV's data, executed second
    virtual void applyConfig( SimulatedIAUV * auv, Vehicle &vehicleChars, SceneBuilder *oscene) = 0;
    //ROSINTERFACE: returns configured ROSInterfaces, executed third
    virtual std::vector<boost::shared_ptr<ROSInterface> > getInterface(ROSInterfaceInfo & rosInterface, std::vector<boost::shared_ptr<SimulatedIAUV> > & iauvFile) = 0;
  
    virtual ~SimulatedDeviceFactory(){};
  };

  //Base class for a simulated device
  class SimulatedDevice{
    std::string type;//driver/rosinterface type
  public:
    std::string name;//common property
    std::string getType(){return type;}
    typedef boost::shared_ptr<SimulatedDevice> Ptr;
    SimulatedDevice(SimulatedDeviceConfig * cfg);
    virtual ~SimulatedDevice(){}
  };
};

#endif /* SIMULATEDDEVICES_H_ */
