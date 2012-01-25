#ifndef URDFROBOT_H_
#define URDFROBOT_H_

#include "SimulatorConfig.h"
#include "KinematicChain.h"
#include "ConfigXMLParser.h"

#include <osgOcean/OceanScene>

#include <iostream>
#include <string.h>

class URDFRobot: public KinematicChain {

public:

	osg::Vec3d * jointAxis;

	URDFRobot(osgOcean::OceanScene *oscene,Vehicle vehicle);

	//virtual osg::Node* setTool(osg::Matrix m, std::string tool_model_file);
	void updateJoints(double *q, int startJoint, int numJoints);

	~URDFRobot();

protected:
	
	void updateJoints(double *q);

};

#endif /* URDFROBOT_H_ */
