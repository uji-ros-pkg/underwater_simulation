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

#include "SimulatedDevices.h"
#include "ConfigXMLParser.h"
#include "ros/ros.h"

//a list of "factories" to initialize and apply a device or rosinterface
//an instance of "config" class is used as both a "factory" and as an XML structure
std::vector<SimulatedDeviceConfig::Ptr> factories;

//setting up a list of available devices/rosinterfaces
static void initFactories(){
	if (factories.size()>0) return;
	//ADD NEW DEVICE/ROSINERFACE BEGIN


	//ADD NEW DEVICE/ROSINERFACE END
	for (size_t i = 0; i< factories.size();++i)
		for (size_t j = 0; j< i;++j)
			if (factories[i]->type == factories[j]->type)
				ROS_ERROR("SimulatedDevices factories types must be unique, but the same type '%s' is specified at indexes %d and %d in initFacotries() in SimulatedDevices.cpp", factories[i]->type.c_str(), (int)j, (int)i);
}

SimulatedDevices::SimulatedDevices(){}

boost::shared_ptr<ROSInterface> SimulatedDevices::getInterface(ROSInterfaceInfo & rosInterface, std::vector<boost::shared_ptr<SimulatedIAUV> > & iauvFile){
	initFactories();
	for (size_t i = 0; i< factories.size();++i)
		if (rosInterface.type==ROSInterfaceInfo::SimulatedDevice &&  factories[i]->type==rosInterface.subtype){
			return factories[i]->getInterface(rosInterface, iauvFile);
		}
	return boost::shared_ptr<ROSInterface>();//empty pointer by default
}

void SimulatedDevices::applyConfig(SimulatedIAUV * auv, Vehicle &vehicleChars)
{
	for (size_t i = 0; i< factories.size();++i){
		factories[i]->applyConfig(auv, vehicleChars);
	}
}

SimulatedDeviceConfig::Ptr SimulatedDevices::processConfig(const xmlpp::Node* node, ConfigFile * config){
	initFactories();
	SimulatedDeviceConfig::Ptr dev;
	std::string name;
	xmlpp::Node::NodeList list = node->get_children();
	for(xmlpp::Node::NodeList::iterator iter = list.begin(); iter != list.end(); ++iter){
		xmlpp::Node* child=dynamic_cast<const xmlpp::Node*>(*iter);

		if (child->get_name()=="name")
			config->extractStringChar(child, name);
		else if (child->get_name()!="text")
			for (size_t i = 0; i< factories.size();++i)
				if (factories[i]->type==child->get_name()){
					dev = factories[i]->processConfig(child, config);
					if (dev){
						//common device XML properties, like name
						dev->name = name;
					}
					return dev;//one device per SimulatedDevice tag, preceded by common properties
				}
	}
	return dev;
}

SimulatedDeviceConfig::SimulatedDeviceConfig(std::string type){
	this->type = type; //device/rosinterface type identifier for both "XML config" and a "factory"
}

SimulatedDevice::SimulatedDevice(SimulatedDeviceConfig * cfg){
	this->type = cfg->type;
	this->name = cfg->name;
}
