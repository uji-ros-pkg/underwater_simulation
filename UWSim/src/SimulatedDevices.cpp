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
#include <osg/Notify>

#include "SimDev_Echo.h"

//a list of "factories" to initialize and apply a device or rosinterface
//an instance of "config" class is used as both a "factory" and as an XML structure
std::vector<SimulatedDeviceFactory::Ptr> factories;

//setting up a list of available devices/rosinterfaces
static void initFactories(){
	if (factories.size()>0) return;
	//ADD NEW DEVICE/ROSINTERFACE BEGIN
	
	factories.push_back(SimulatedDeviceFactory::Ptr(new SimDev_Echo_Factory()));
	
	//ADD NEW DEVICE/ROSINTERFACE END
	for (size_t i = 0; i< factories.size();++i)
		for (size_t j = 0; j< i;++j)
			if (factories[i]->getType() == factories[j]->getType())
				OSG_FATAL<<"SimulatedDevices factories types must be unique, but the same type '"<<factories[i]->getType()
				<<"' is specified at indexes "<<j<<" and "<<i<<" in initFacotries() in SimulatedDevices.cpp" << std::endl;
}

SimulatedDevices::SimulatedDevices(){}

std::vector<boost::shared_ptr<ROSInterface> >
SimulatedDevices::getInterfaces(ROSInterfaceInfo & rosInterface, std::vector<boost::shared_ptr<SimulatedIAUV> > & iauvFile){
	initFactories();
	std::vector<boost::shared_ptr<ROSInterface> > ifaces;
	bool isFactoryFound = false;
	if (rosInterface.type==ROSInterfaceInfo::SimulatedDevice){
		for (size_t i = 0; i< factories.size();++i)
			if (factories[i]->getType()==rosInterface.subtype){
				isFactoryFound = true;
				std::vector<boost::shared_ptr<ROSInterface> > ifaces_ = factories[i]->getInterface(rosInterface, iauvFile);
				for (size_t j = 0; j< ifaces_.size();++j)
					ifaces.push_back(ifaces_[j]);
			}
			
		if(!isFactoryFound)
			OSG_FATAL<<"Unknown ROSIterface '"<<rosInterface.subtype<<"ROS', skipping..."<<std::endl;
	}
	return ifaces;
}

void SimulatedDevices::applyConfig(SimulatedIAUV * auv, Vehicle &vehicleChars, SceneBuilder *oscene)
{
	for (size_t i = 0; i< factories.size();++i){
		factories[i]->applyConfig(auv, vehicleChars, oscene);
	}
}

static void processConfigNode(const xmlpp::Node* node, ConfigFile * config, SimulatedDeviceConfig::Ptr cfg)
{
	if (!cfg) return;
	
	xmlpp::Node::NodeList list = node->get_children();
	for(xmlpp::Node::NodeList::iterator iter = list.begin(); iter != list.end(); ++iter){
		xmlpp::Node* child=dynamic_cast<const xmlpp::Node*>(*iter);
		if (child->get_name()=="name" && cfg->name.length()==0)
			config->extractStringChar(child, cfg->name);
	}
}

std::vector< SimulatedDeviceConfig::Ptr >
SimulatedDevices::processConfig(const xmlpp::Node* node, ConfigFile * config){
	initFactories();
	std::vector< SimulatedDeviceConfig::Ptr > devs;
	xmlpp::Node::NodeList list = node->get_children();
	for(xmlpp::Node::NodeList::iterator iter = list.begin(); iter != list.end(); ++iter){
		xmlpp::Node* child=dynamic_cast<const xmlpp::Node*>(*iter);
		
		if (child->get_name()!="text"){
			bool isFactoryFound = false;
			for (size_t i = 0; i< factories.size();++i)
				if (factories[i]->getType()==child->get_name()){
					isFactoryFound = true;
					SimulatedDeviceConfig::Ptr dev = factories[i]->processConfig(child, config);
					if (dev)
						processConfigNode(child, config, dev);
					devs.push_back(dev);
				}
			if (!isFactoryFound)
				OSG_FATAL<<"Unknown SimulatedDevice '"<<child->get_name()<<"', skipping..."<<std::endl;
		}
	}
	return devs;
}

SimulatedDeviceConfig::SimulatedDeviceConfig(std::string type){
	this->type = type;
}

SimulatedDeviceFactory::SimulatedDeviceFactory(std::string type){
	this->type = type;
}

SimulatedDevice::SimulatedDevice(SimulatedDeviceConfig * cfg){
	this->type = cfg->getType();
	this->name = cfg->name;
}
