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

void KinematicChain::setFullJointPosition(double *newq, int n) {
	for(int i=0;i<getNumberOfJoints();i++){ 
	    if(newq[i]<limits[i].first)
	      q[i]=limits[i].first;
	    else if(newq[i]>limits[i].second)
	      q[i]=limits[i].second;
	    else {
	      if (!isnan(q[i]))
	      	q[i]=newq[i];
	      else {
		OSG_FATAL << "KinematicChain::setJointPosition received NaN" << std::endl;
	      }
	    }
	}
	updateJoints(q);
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
	
void KinematicChain::setFullJointVelocity(double *qdot, int n) {
	for (int i=0; i<getNumberOfJoints(); i++){
	    if(q[i]+qdot[i]<limits[i].first)
	      q[i]=limits[i].first;
	    else if(q[i]+qdot[i]>limits[i].second)
	      q[i]=limits[i].second;
	    else
	      q[i]+=qdot[i];
	}
	updateJoints(q);
}

void KinematicChain::setJointVelocity(double *qdot, int n) {
	int offset=0;
	for (int i=0; i<getNumberOfJoints(); i++){
	  if (i-offset>=n) break;

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


void KinematicChain::setFullJointPosition(std::vector<double> &q) {
	setFullJointPosition(&(q.front()), q.size());
}

void KinematicChain::setFullJointVelocity(std::vector<double> &qdot) {
	setFullJointPosition(&(qdot.front()), qdot.size());
}

void KinematicChain::setJointPosition(std::vector<double> &q) {
	setJointPosition(&(q.front()), q.size());
}

void KinematicChain::setJointVelocity(std::vector<double> &qdot) {
	setJointPosition(&(qdot.front()), qdot.size());
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
