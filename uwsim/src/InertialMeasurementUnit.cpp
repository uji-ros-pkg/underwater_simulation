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

#include "UWSimUtils.h"
#include "InertialMeasurementUnit.h"

osg::Quat InertialMeasurementUnit::getMeasurement() {
	//Should get world coords and then transform to the localizedWorld
	osg::Matrixd *rMi=getWorldCoords(imu_node_);
	osg::Matrixd lMi=*rMi*osg::Matrixd::inverse(rMl_);

	//Now add some gaussian noise
	static boost::normal_distribution<> normal(0,std_);
	static boost::variate_generator<boost::mt19937&, boost::normal_distribution<> > var_nor(rng_, normal);

	return lMi.getRotate()*osg::Quat(var_nor(), osg::Vec3d(1,0,0), var_nor(), osg::Vec3d(0,1,0), var_nor(), osg::Vec3d(0,0,1));
}
