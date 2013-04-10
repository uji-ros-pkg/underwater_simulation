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

#ifndef PRESSURESENSOR_H_
#define PRESSURESENSOR_H_

#include <osg/Vec3d>
#include <osg/Node>
#include <osg/Matrix>
#include <osg/Group>

#include <boost/random.hpp>

class PressureSensor {

public:
    std::string name;

    /** Constructor
     * @param name the name of the pressure sensor
     * @param parent the node of the scene graph that holds the sensor
     * @param rMl the sensor measures are given with respect to the root (r). Use rMl to transform them to another frame ('l' is the new frame, typically the localized world)
     * @param std the standard deviation on the sensor measures
     */
    PressureSensor(std::string sensor_name, osg::Node *parent, osg::Matrixd rMl, double std=0): name(sensor_name), parent_(parent), rMl_(rMl), std_(std) {
    	node_=new osg::Node();
    	parent->asGroup()->addChild(node_);
    }

    double getMeasurement();

    double getStandardDeviation() {return std_;}

    virtual ~PressureSensor() {}

private:
    osg::ref_ptr<osg::Node> parent_;
    osg::Matrixd rMl_;
    double std_;
    osg::ref_ptr<osg::Node> node_;

    boost::mt19937 rng_; ///< Boost random number generator
};


#endif /* PRESSURESENSOR_H_ */
