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

#include "SimulatedDevice_EchoImpl.h"

SimulatedDevice_Echo::SimulatedDevice_Echo(SimulatedDeviceConfig_Echo * cfg): SimulatedDevice(cfg) {
	this->info = cfg->info;
}


SimulatedDeviceConfig_Echo::SimulatedDeviceConfig_Echo(): SimulatedDeviceConfig("echo") {

}

SimulatedDeviceConfig::Ptr SimulatedDeviceConfig_Echo::processConfig(const xmlpp::Node* node, ConfigFile * config){
	SimulatedDeviceConfig_Echo * cfg = new SimulatedDeviceConfig_Echo();
	xmlpp::Node::NodeList list = node->get_children();
	for(xmlpp::Node::NodeList::iterator iter = list.begin(); iter != list.end(); ++iter){
		xmlpp::Node* child=dynamic_cast<const xmlpp::Node*>(*iter);
		//std::cout << "cn="<<child->get_name()<<std::endl;
		if(child->get_name()=="info")
			config->extractStringChar(child,cfg->info);
		//std::cout << "info="<<this->info<<std::endl;
	}
	return SimulatedDeviceConfig::Ptr(cfg);
}

void SimulatedDeviceConfig_Echo::applyConfig( SimulatedIAUV * auv, Vehicle &vehicleChars){
	for (size_t i=0; i< vehicleChars.simulated_devices.size(); ++i)
		if (vehicleChars.simulated_devices[i]->type == this->type){
			SimulatedDeviceConfig_Echo * cfg = static_cast<SimulatedDeviceConfig_Echo *>(vehicleChars.simulated_devices[i].get());
			//std::cout << "info2="<<this->info<<", info3="<<cfg->info<<std::endl;
			if (cfg->info.length()>0){
				auv->devices->all.push_back(SimulatedDevice_Echo::Ptr(new SimulatedDevice_Echo(cfg)));
			}
			else
				ROS_WARN("Echo device '%s' inside robot '%s' has empty info, discarding...", vehicleChars.simulated_devices[i]->name.c_str(),vehicleChars.name.c_str());
		}
}

boost::shared_ptr<ROSInterface> SimulatedDeviceConfig_Echo::getInterface(ROSInterfaceInfo & rosInterface, std::vector<boost::shared_ptr<SimulatedIAUV> > & iauvFile){
	//return boost::shared_ptr<ROSInterface>( new SimulatedDeviceROS_Echo(NULL, rosInterface.topic, rosInterface.rate) );//it is possible to have ROS interface without any device
	for (size_t i=0;i<iauvFile.size();++i)
		for (size_t d=0;d<iauvFile[i]->devices->all.size();++d)
			if (iauvFile[i]->devices->all[d]->type == this->type && iauvFile[i]->devices->all[d]->name == rosInterface.targetName)
				return boost::shared_ptr<ROSInterface>( new SimulatedDeviceROS_Echo(static_cast<SimulatedDevice_Echo*>(iauvFile[i]->devices->all[d].get()), rosInterface.topic, rosInterface.rate) );
	ROS_WARN("Returning empty ROS interface for device %s...", rosInterface.targetName.c_str());
	return boost::shared_ptr<ROSInterface>();
}

SimulatedDeviceROS_Echo::SimulatedDeviceROS_Echo(SimulatedDevice_Echo *dev, std::string topic, int rate): ROSPublisherInterface(topic,rate), dev(dev) {
}

void  SimulatedDeviceROS_Echo::createPublisher(ros::NodeHandle &nh) {
  ROS_INFO("SimulatedDeviceROS_Echo publisher on topic %s",topic.c_str());
  pub_ = nh.advertise<std_msgs::String>(topic, 1);
}

void  SimulatedDeviceROS_Echo::publish() {
	std_msgs::String msg;
	if (dev!=NULL)
		msg.data = dev->info;
	else
		msg.data = "dev==NULL";
	pub_.publish(msg);
}

SimulatedDeviceROS_Echo::~SimulatedDeviceROS_Echo() {}
