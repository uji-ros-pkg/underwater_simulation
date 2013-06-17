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

/*
 * Example header of driver/rosinterface implementation
 *
 * Included only in driver's cpp file
 */
#ifndef SIMULATEDDEVICE_ECHOIMPL_H_
#define SIMULATEDDEVICE_ECHOIMPL_H_
#include "SimulatedDevice_Echo.h"
#include "ConfigXMLParser.h"
#include "SimulatedIAUV.h"
#include "ROSInterface.h"
#include <ros/ros.h>
#include <std_msgs/String.h>

//Driver class
class SimulatedDevice_Echo: public SimulatedDevice {
public:
	std::string info;	//Device's property

	SimulatedDevice_Echo(SimulatedDeviceConfig_Echo * cfg);
};

//ROSInterface class
class SimulatedDeviceROS_Echo : public ROSPublisherInterface {
	//this is just an example, use a pointer to SimulatedIAUV, if only ROSInterface is implemented
	//pointer to a device
	SimulatedDevice_Echo * dev;
public:
	SimulatedDeviceROS_Echo(SimulatedDevice_Echo *dev, std::string topic, int rate);

	void createPublisher(ros::NodeHandle &nh);
	void publish();

	~SimulatedDeviceROS_Echo();
};

#endif /* SIMULATEDDEVICE_ECHOIMPL_H_ */
