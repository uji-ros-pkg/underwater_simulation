#ifndef URDFROBOT_H_
#define URDFROBOT_H_

#include "SimulatorConfig.h"
#include "KinematicChain.h"
#include "ConfigXMLParser.h"

#include <osgOcean/OceanScene>
#include <osg/Switch>

#include <iostream>
#include <string.h>

class URDFRobot: public KinematicChain {

public:

	std::vector<osg::Vec3d> joint_axis;


	URDFRobot(osgOcean::OceanScene *oscene,Vehicle vehicle);

	~URDFRobot();

protected:
	
	void updateJoints(std::vector<double> &q);
	void updateJoints(std::vector<double> &q, int startJoint, int numJoints);

};

#endif /* URDFROBOT_H_ */
