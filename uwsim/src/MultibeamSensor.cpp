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

#include <uwsim/MultibeamSensor.h>
#include <osg/PositionAttitudeTransform>

MultibeamSensor::MultibeamSensor(osg::Group *uwsim_root, std::string name, std::string parentName, osg::Node *trackNode, double initAngle,
                                 double finalAngle, double alpha, double range, unsigned int mask, int visible,unsigned int ARMask)
{


  osg::PositionAttitudeTransform * mTc= new osg::PositionAttitudeTransform;
  mTc->setPosition(osg::Vec3d(0,0,0));
  mTc->setAttitude(osg::Quat( (finalAngle + initAngle)/2 * M_PI /180.0, osg::Vec3d(1,0,0)));
  trackNode->asTransform()->addChild(mTc);
  vcam = VirtualCamera(uwsim_root, name,parentName, mTc, fabs(finalAngle - initAngle) / alpha + 1, fabs(finalAngle - initAngle), range);

  this->numpixels = fabs(finalAngle - initAngle) / alpha + 1;
  this->range = range;
  this->initAngle = initAngle;
  this->finalAngle = finalAngle;
  this->angleIncr = alpha;
  this->name=name;
  this->trackNode = trackNode;
  parentLinkName=parentName;
  preCalcTable();
  vcam.textureCamera->setCullMask(mask);

  if (visible)
  {
    osg::ref_ptr<osg::Geometry> beam = osg::ref_ptr<osg::Geometry>(new osg::Geometry);
    osg::ref_ptr<osg::Vec3Array> points = new osg::Vec3Array;
    for(double initAux=initAngle;initAux<finalAngle;initAux+=angleIncr)
    {
      osg::Vec3d start(0, 0, 0);
      osg::Vec3d end(0, sin(initAux*3.14/180.0)*range, -cos(initAux*3.14/180.0)*range);
      points->push_back(start);
      points->push_back(end);
    }
    osg::ref_ptr<osg::Vec4Array> color = new osg::Vec4Array;
    color->push_back(osg::Vec4(0.0, 1.0, 0.0, 0.6));
    beam->setVertexArray(points.get());
    beam->setColorArray(color.get());
    beam->setColorBinding(osg::Geometry::BIND_OVERALL);
    beam->addPrimitiveSet(new osg::DrawArrays(GL_LINES, 0, points->size()));
    geode = osg::ref_ptr<osg::Geode>(new osg::Geode());
    geode->addDrawable(beam.get());
    geode->setNodeMask(ARMask);
  }
  trackNode->asGroup()->addChild(geode);
}

void MultibeamSensor::preCalcTable()
{

  //Create matrix to unproject camera points to real world
  osg::Matrix *MVPW = new osg::Matrix(
      vcam.textureCamera->getViewMatrix() * vcam.textureCamera->getProjectionMatrix()
          * vcam.textureCamera->getViewport()->computeWindowMatrix());
  MVPW->invert(*MVPW);

  //Get real fov from camera
  osg::Vec3d first = osg::Vec3d(0, 0, 1) * (*MVPW), last = osg::Vec3d(0, numpixels - 1, 1) * (*MVPW), center =
                 osg::Vec3d(0, numpixels / 2, 1) * (*MVPW);
  double realfov = acos((first * last) / (last.length() * first.length()));
  double thetacenter = acos((first * center) / (center.length() * first.length()));
  double alpha = realfov / (numpixels);
  //std::cout<<realfov<<" "<<alpha<<std::endl;

  //Interpolate points
  remapVector.resize(numpixels);
  int current = 0;
  double lastTheta = 0;
  for (int i = 0; i < numpixels; i++)
  {
    osg::Vec3d point = osg::Vec3d(0, i, 1) * (*MVPW);

    double theta = acos((first * point) / (first.length() * point.length()));
    while (theta >= alpha * current && current < numpixels)
    {
      if (theta == alpha * current)
      { //usually only first iteration as point has to be exactly the same
        remapVector[current].pixel1 = i;
        remapVector[current].weight1 = 0.50;
        remapVector[current].pixel2 = i;
        remapVector[current].weight2 = 0.50;
      }
      else
      { //Interpolate between this and last point
        double dist = fabs(theta - alpha * current), prevdist = fabs(lastTheta - alpha * current);
        remapVector[current].pixel1 = i;
        remapVector[current].weight1 = prevdist / (dist + prevdist);
        remapVector[current].pixel2 = i - 1;
        remapVector[current].weight2 = dist / (dist + prevdist);
        //std::cout<<remapVector[current].weight1<<" "<<remapVector[current].weight2<<" "<<remapVector[current].weight1+remapVector[current].weight2<<std::endl;
      }
      remapVector[current].distort = 1 / cos(fabs(theta - thetacenter));
      //std::cout<<"remap: "<<remapVector[current].distort<<std::endl;
      current++;
      //std::cout<<theta<<":"<<tan(theta)<<" asd:"<<fx<<std::endl;
      //std::cout<<current<<" "<<i<<std::endl;
    }
    lastTheta = theta;
    //std::cout<<" THETA: "<<theta<<"Current point: "<<current*alpha<<"Error: "<<theta-i*alpha<<"asd: "<<current<<std::endl;
  }

}

int MultibeamSensor::getTFTransform(tf::Pose & pose, std::string & parent){
  parent=parentLinkName;
  pose.setOrigin(tf::Vector3(trackNode->asTransform()->asPositionAttitudeTransform()->getPosition().x(),
                        trackNode->asTransform()->asPositionAttitudeTransform()->getPosition().y(),
                        trackNode->asTransform()->asPositionAttitudeTransform()->getPosition().z()));
  pose.setRotation( tf::Quaternion(trackNode->asTransform()->asPositionAttitudeTransform()->getAttitude().x(),
                        trackNode->asTransform()->asPositionAttitudeTransform()->getAttitude().y(),
                        trackNode->asTransform()->asPositionAttitudeTransform()->getAttitude().z(),
                        trackNode->asTransform()->asPositionAttitudeTransform()->getAttitude().w()));

  tf::Pose OSGToTFconvention;
  OSGToTFconvention.setRotation(tf::Quaternion(tf::Vector3(0,1,0),M_PI/2));  //As we are using camera to simulate it, we need to rotate it
  pose=pose*OSGToTFconvention;

  return 1;

}
