/* 
 * Copyright (c) 2013 University of Jaume-I.
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the GNU Public License v3.0
 * which accompanies this distribution, and is available at
 * http://www.gnu.org/licenses/gpl.html
 * 
 * Contributors:
 *     Mario Prats
 *     Javier Perez
 */ 

#include <uwsim/VirtualRangeSensor.h>
#include <uwsim/UWSimUtils.h>
#include <iostream>

VirtualRangeSensor::VirtualRangeSensor(){}

VirtualRangeSensor::VirtualRangeSensor(std::string name, osg::Node *root, osg::Node *trackNode, double range, bool visible) {
	init(name, root, trackNode, range, visible);
}

void VirtualRangeSensor::init(std::string name, osg::Node *root, osg::Node *trackNode, double range, bool visible) {
	this->name=name;
	this->root=root;
	
	this->trackNode=trackNode;
	//Add a switchable frame geometry on the sensor frame
        osg::ref_ptr<osg::Node> axis=UWSimGeometry::createSwitchableFrame();
	this->trackNode->asGroup()->addChild(axis);
	
	this->range=range;
	this->visible=visible;

	//make this virtual ray track the node
	node_tracker = new IntersectorUpdateCallback(range,visible,root);
	trackNode->setUpdateCallback(node_tracker);
	trackNode->asGroup()->addChild(node_tracker->geode);
}




