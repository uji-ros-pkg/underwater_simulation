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

#ifndef SIMULATEDDEVICE_ECHO_H_
#define SIMULATEDDEVICE_ECHO_H_
#include "SimulatedDevices.h"

/*
 * Example header of driver/rosinterface configuration/factory
 *
 * Included in SimulatedDevices.cpp
 */

//Driver/ROSInterface configuration/factory class
class SimulatedDeviceConfig_Echo : public SimulatedDeviceConfig {
public:
//XML members
	std::string info;
//constructor
	SimulatedDeviceConfig_Echo();
//Factory methods
//for pure ROSInterface implement only getInterface
//for pure Driver implement only processConfig && applyConfig members

	SimulatedDeviceConfig::Ptr processConfig(const xmlpp::Node* node, ConfigFile * config);
	void applyConfig( SimulatedIAUV * auv, Vehicle &vehicleChars);
	boost::shared_ptr<ROSInterface> getInterface(ROSInterfaceInfo & rosInterface, std::vector<boost::shared_ptr<SimulatedIAUV> > & iauvFile);
};

#endif /* SIMULATEDDEVICE_ECHO_H_ */
