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
}

void KinematicChain::setJointPosition(double *newq) {
	int offset=0;
	for(int i=0;i<getNumberOfJoints();i++){
	  if(jointType[i]==0 || mimic[i].joint!=i){
	    offset++;
	    q[i]=0;
	  }
	  else{
	    if(newq[i-offset]<limits[i].first)
	      q[i]=limits[i].first;
	    else if(newq[i-offset]>limits[i].second)
	      q[i]=limits[i].second;
	    else
	      q[i]=newq[i-offset];
	  }
	}
	updateJoints(q);
}
	
void KinematicChain::setJointVelocity(double *qdot) {
	int offset=0;
	for (int i=0; i<getNumberOfJoints(); i++){
	  if(jointType[i]==0 || mimic[i].joint!=i)
	    offset++;
	  else{
	    if(q[i]+qdot[i-offset]<limits[i].first)
	      q[i]=limits[i].first;
	    else if(q[i]+qdot[i-offset]>limits[i].second)
	      q[i]=limits[i].second;
	    else
	      q[i]+=qdot[i-offset];
	  }
	}
	updateJoints(q);
}

void KinematicChain::setJointPosition(std::vector<double> &q) {
	setJointPosition(&(q.front()));
}

void KinematicChain::setJointVelocity(std::vector<double> &qdot) {
	setJointPosition(&(qdot.front()));
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
