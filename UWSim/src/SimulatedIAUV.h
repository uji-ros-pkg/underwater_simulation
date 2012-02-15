/*
 * SimulatedIAUV.h
 *
 *  Created on: 03/05/2010
 *      Author: mprats
 */

#ifndef SIMULATEDIAUV_H_
#define SIMULATEDIAUV_H_

#include "osgOceanScene.h"
#include "URDFRobot.h"
#include "VirtualCamera.h"
#include "ConfigXMLParser.h"
#include "VirtualRangeSensor.h"


/* An I-AUV */
class SimulatedIAUV
{
public:
	VirtualCamera *camview;
	VirtualRangeSensor *range_sensors;

	char ncams;
	char n_range_sensors;
	typedef enum {ARM5,PA10} arm_t;

	std::string name;		///< Vehicle name
	URDFRobot *urdf;		///< URDF I-AUV
	osg::LightSource* lightSource;	///< vehicle lamp
	osg::MatrixTransform *baseTransform;

	//SimulatedIAUV(osgOcean::OceanScene *oscene, arm_t armtype);
        SimulatedIAUV(osgOceanScene *oscene, Vehicle vehicle);

	//void setVirtualCamera(std::string name, osg::Transform* transform, osg::Vec3d eye, osg::Vec3d orientation, osg::Vec3d upVector, int width, int height);

	//setPosition
	void setVehiclePosition(double x, double y, double z, double yaw) {setVehiclePosition(x,y,z,0,0,yaw);}
	void setVehiclePosition(double x, double y, double z, double roll, double pitch, double yaw);
	void setVehiclePosition(double p[6]) {setVehiclePosition(p[0],p[1],p[2], p[3], p[4], p[5]);}
	void setVehiclePosition(osg::Matrixd m);

	~SimulatedIAUV() {
		if (urdf!=NULL) delete urdf;
		if (camview!=NULL) delete camview;
	}
};

#endif /* SIMULATEDIAUV_H_ */
