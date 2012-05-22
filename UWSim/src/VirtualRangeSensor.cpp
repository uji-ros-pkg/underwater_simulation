/*
 * VirtualRangeSensor.cpp
 *
 *  Created on: 11/01/2012
 *      Author: mprats
 */

#include "VirtualRangeSensor.h"
#include "UWSimUtils.h"
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




