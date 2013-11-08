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

#ifndef MULTIBEAMSENSOR_H_
#define MULTIBEAMSENSOR_H_

#include "VirtualCamera.h"

class MultibeamSensor : public VirtualCamera
{
  struct Remap
  {
    int pixel1, pixel2;
    double weight1, weight2;
    double distort;
  };

public:
  int numpixels;
  double range, initAngle, finalAngle, angleIncr;
  std::vector<Remap> remapVector;
  MultibeamSensor(osg::Group *uwsim_root, std::string name, osg::Node *trackNode, double initAngle, double finalAngle,
                  double alpha, double range);
  void preCalcTable();
};

#endif
