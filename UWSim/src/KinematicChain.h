/*
 * KinematicChain.h
 *
 *  Created on: 22/11/2011
 *      Author: mprats
 */

#ifndef KINEMATICCHAIN_H_
#define KINEMATICCHAIN_H_

#include "SimulatorConfig.h"
#include <osg/MatrixTransform>

#include <iostream>
#include <string.h>


struct MimicArm{
	  int joint;
	  double offset,multiplier;
	  int sliderCrank;
};

/** Abstract Kinematic Chain class for holding articulated 3D Models
 */
class KinematicChain {
public:
	osg::ref_ptr<osg::Node> *link;	///< pointers to link models
	int nlinks;			///< number of links
	int njoints;			///< number of joints
	double *q;			///< Joint values
	double **limits;		//Limits for joints by default -PI , PI

	MimicArm * mimic;		//Mimic joints info
	int *jointType;			//type of joints 0 fixed, 1 rotation, 2 prismatic
	osg::MatrixTransform **joints;	///< pointers to transforms between links
	osg::MatrixTransform **zerojoints; ///<pointers to original (zero) transforms between links
	osg::MatrixTransform* baseTransform; ///<pointer to the first node in the PA10 graph (base tranform)

	//osg::MatrixTransform *tool_transform;	///< Transform between the end-effector and the tool base frame
	//osg::ref_ptr<osg::Node> tool;		///< Pointer to the tool osg node


	KinematicChain(int nlinks, int njoints);

	//setJointPosition, setJointVelocity, etc.
	void setJointPosition(double *q);
	void setJointVelocity(double *qdot);
	std::vector<double> getJointPosition();

	//virtual osg::Node* setTool(osg::Matrix m, std::string tool_model_file);

	~KinematicChain();

protected:
	virtual void updateJoints(double *q)=0;	///< Implemented by childs for joint position update
};

#endif /* KINEMATICCHAIN_H_ */
