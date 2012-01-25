/*
 * KinematicChain.cpp
 *
 *  Created on: 22/11/2011
 *      Author: mprats
 */

#include "KinematicChain.h"
#include <osgDB/Registry>
#include <osgDB/ReadFile>

KinematicChain::KinematicChain(int nlinks, int njoints) {
	this->nlinks=nlinks;
	this->njoints=njoints;

	//tool_transform=NULL;
	//tool=NULL;

	jointType = new int[njoints];
	mimic= new MimicArm[njoints];
	limits= new double*[njoints];
	for(int i=0;i<njoints;i++){
	  limits[i]= new double[2];
	}
	q=new double[njoints];
	memset(q,0,njoints*sizeof(double));
}

void KinematicChain::setJointPosition(double *newq) {
	//memcpy(q,newq,njoints*sizeof(double));

	int offset=0;
	for(int i=0;i<njoints;i++){
	  if(jointType[i]==0 || mimic[i].joint!=i){
	    offset++;
	    q[i]=0;
	  }
	  else{
	    if(newq[i-offset]<limits[i][0])
	      q[i]=limits[i][0];
	    else if(newq[i-offset]>limits[i][1])
	      q[i]=limits[i][1];
	    else
	      q[i]=newq[i-offset];
	  }
	}
	updateJoints(q);
}
	
void KinematicChain::setJointVelocity(double *qdot) {

	int offset=0;
	for (int i=0; i<njoints; i++){
	  if(jointType[i]==0 || mimic[i].joint!=i)
	    offset++;
	  else{
	    if(q[i]+qdot[i-offset]<limits[i][0])
	      q[i]=limits[i][0];
	    else if(q[i]+qdot[i-offset]>limits[i][1])
	      q[i]=limits[i][1];
	    else
	      q[i]+=qdot[i-offset];
	  }

	}
	updateJoints(q);
}

std::vector<double> KinematicChain::getJointPosition() {
	std::vector<double> validq;
	for(int i=0;i<njoints;i++){
	  if(jointType[i]!=0 && mimic[i].joint==i) 
	    validq.push_back(q[i]);
	}
	return validq;
}

/*
osg::Node* KinematicChain::setTool(osg::Matrix m, std::string tool_model_file) {
	osgDB::Registry::instance()->getDataFilePathList().push_back(std::string(SIMULATOR_DATA_PATH)+std::string("/objects"));
	osgDB::Registry::instance()->getDataFilePathList().push_back(std::string(SIMULATOR_DATA_PATH)+std::string("/robot"));
	this->tool_transform=new osg::MatrixTransform(m);
	this->tool=osgDB::readNodeFile(tool_model_file);

	if (link!=NULL) {
		link[njoints-1]->asGroup()->addChild(tool_transform);
		tool_transform->addChild(tool);
	}
	return tool;
}
*/

KinematicChain::~KinematicChain() {
	delete [] q;
}
