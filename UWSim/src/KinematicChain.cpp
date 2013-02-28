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
	jointType.resize(njoints);
	mimic.resize(njoints);
	limits.resize(njoints);
	q.resize(njoints);
	memset(&(q.front()),0,njoints*sizeof(double));
	started=0;
}

void KinematicChain::setJointPosition(double *newq, int n) {
	int offset=0;
	for(int i=0;i<getNumberOfJoints();i++){
	  if (i-offset>=n) break;

	  if(jointType[i]==0 || mimic[i].joint!=i){
	    offset++;
	    q[i]=0;
	  }
	  else{
	    if(newq[i-offset]<limits[i].first)
	      q[i]=limits[i].first;
	    else if(newq[i-offset]>limits[i].second)
	      q[i]=limits[i].second;
	    else {
	      if (!isnan(q[i]))
	      	q[i]=newq[i-offset];
	      else {
		std::cerr << "KinematicChain::setJointPosition received NaN" << std::endl;
		OSG_FATAL << "KinematicChain::setJointPosition received NaN" << std::endl;
	      }
	    }
	  }
	}
	updateJoints(q);
}
	
void KinematicChain::setJointVelocity(double *qdot, int n) {
	//Time issues
    double elapsed=0;
    if (started!=0) {
      ros::WallDuration t_diff = ros::WallTime::now() - last;
      elapsed= t_diff.toSec();
      //If elapsed>MAX_ELAPSED, consider this is sent by a different publisher, so that the counter has to restart
      if (elapsed>1) elapsed=0;
    }	
    
    started=1;
    last = ros::WallTime::now();
	
	int offset=0;
	for (int i=0; i<getNumberOfJoints(); i++){
	  if (i-offset>=n) break;

	  if(jointType[i]==0 || mimic[i].joint!=i)
	    offset++;
	  else{
	    if(q[i]+(qdot[i-offset]*elapsed)<limits[i].first)
	      q[i]=limits[i].first;
	    else if(q[i]+(qdot[i-offset]*elapsed)>limits[i].second)
	      q[i]=limits[i].second;
	    else
	      q[i]+=qdot[i-offset]*elapsed;
	  }
	}
	updateJoints(q);
}

void KinematicChain::setJointPosition(std::vector<double> &q) {
	setJointPosition(&(q.front()), q.size());
}

void KinematicChain::setJointVelocity(std::vector<double> &qdot) {
	setJointVelocity(&(qdot.front()), qdot.size());
}

std::vector<double> KinematicChain::getJointPosition() {
	std::vector<double> validq;
	for(int i=0;i<getNumberOfJoints();i++){
	  if(jointType[i]!=0 && mimic[i].joint==i) 
	    validq.push_back(q[i]);
	}
	return validq;
}

KinematicChain::~KinematicChain() {}
