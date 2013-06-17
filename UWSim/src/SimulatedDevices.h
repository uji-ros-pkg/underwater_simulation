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

#ifndef SIMULATEDDEVICES_H_
#define SIMULATEDDEVICES_H_

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

//Base class for device/rosinterface "factory" and device's XML configuration
class SimulatedDeviceConfig{
public:
	typedef boost::shared_ptr<SimulatedDeviceConfig> Ptr;

	std::string type;	//device/rosinterface type identifier for both "XML config" and a "factory"
	SimulatedDeviceConfig(std::string type);

//common XML properties:
	std::string name;

//factory methods

	//DRIVER: parses XML and returns "XML config", executed first
	virtual SimulatedDeviceConfig::Ptr processConfig(const xmlpp::Node* node, ConfigFile * config) = 0;
	//DRIVER: checks parsed XML configurations and sets SimulatedAUV's data, executed second
	virtual void applyConfig( SimulatedIAUV * auv, Vehicle &vehicleChars) = 0;
	//ROSINTERFACE: returns configured ROSInterface, executed third
	virtual boost::shared_ptr<ROSInterface> getInterface(ROSInterfaceInfo & rosInterface, std::vector<boost::shared_ptr<SimulatedIAUV> > & iauvFile) = 0;

	virtual ~SimulatedDeviceConfig(){};
};

//Base class for a simulated device
class SimulatedDevice{
public:
	typedef boost::shared_ptr<SimulatedDevice> Ptr;

	std::string type;//driver/rosinterface type
	std::string name;//common property

	SimulatedDevice(SimulatedDeviceConfig * cfg);
};

//Class added to SimulatedIAUV as devices->, put your data into into this class members or use all vector to store your device
class SimulatedDevices{
public:
//Object members
	std::vector<SimulatedDevice::Ptr> all;

	SimulatedDevices();

	//Applies parsed XML configurations by calling appropriate methods
	//on all registered factories (set in initFactories() in SimulatedDevices.cpp)
	void applyConfig(SimulatedIAUV * auv, Vehicle &vehicleChars);

//Factory methods

	//returns configured ROSInterface based on given XML configuration
	static boost::shared_ptr<ROSInterface> getInterface(ROSInterfaceInfo & rosInterface, std::vector<boost::shared_ptr<SimulatedIAUV> > & iauvFile);

	//Parses driver's XML configuration
	static SimulatedDeviceConfig::Ptr processConfig(const xmlpp::Node* node, ConfigFile * config);
};

#endif /* SIMULATEDDEVICES_H_ */
