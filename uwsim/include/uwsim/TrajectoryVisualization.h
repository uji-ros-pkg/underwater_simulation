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

#ifndef TRAJECTORYVISUALIZATION_H_
#define TRAJECTORYVISUALIZATION_H_

#include "UWSimUtils.h"
#include <osg/NodeTrackerCallback>
#include <osg/LineStipple>


//Node tracker that updates vehicle trajectory visualization.
class TrajectoryUpdateCallback : public osg::NodeTrackerCallback
{
  virtual void operator()(osg::Node *node, osg::NodeVisitor *nv)
  {
    boost::shared_ptr<osg::Matrix> objectMat= getWorldCoords(node);
    osg::Matrixd  res=*objectMat * *LWMat;
    if (trajectory_initialized)
    {
      if ((trajectory_points->back() - res.getTrans()).length() > maxWaypointDistance)
      {
        trajectory_points->push_back(res.getTrans());
        trajectory->setVertexArray(trajectory_points);
        ((osg::DrawArrays*)prset)->setFirst(0);
        ((osg::DrawArrays*)prset)->setCount(trajectory_points->size());
        //std::cerr << "Trajectory_points size: " << trajectory_points->size() << std::endl;
      }
    }
    else
    {
      trajectory_points->clear();
      trajectory_points->push_back(res.getTrans());
      trajectory_initialized = true;
    }
    traverse(node, nv);
  }
public:
  
  bool trajectory_initialized;
  osg::Vec3Array *trajectory_points;
  osg::ref_ptr<osg::Geometry> trajectory;
  osg::PrimitiveSet *prset;
  double maxWaypointDistance;

  osg::ref_ptr<osg::Geode> geode; //Geometry node that draws the beam
  boost::shared_ptr<osg::Matrix> LWMat; //LocalizedWorld Inverted Matrix ( to know distnace from it)

  TrajectoryUpdateCallback(double color[3], double maxWaypointDistance, int pattern, osg::Group *rootNode)
  {
    this->maxWaypointDistance=maxWaypointDistance;
    trajectory_initialized=false;
    trajectory_points = new osg::Vec3Array;
    trajectory_points->push_back(osg::Vec3(0, 0, 0));
    trajectory = osg::ref_ptr < osg::Geometry > (new osg::Geometry());
    trajectory->setVertexArray(trajectory_points);

    osg::Vec4Array* colors = new osg::Vec4Array;
    colors->push_back(osg::Vec4f(color[0], color[1], color[2], 1.0f));
    trajectory->setColorArray(colors);
    trajectory->setColorBinding(osg::Geometry::BIND_OVERALL);
    prset = new osg::DrawArrays(osg::PrimitiveSet::LINE_STRIP);
    trajectory->addPrimitiveSet(prset);
    trajectory->setUseDisplayList(false);

    geode = osg::ref_ptr < osg::Geode > (new osg::Geode());
    geode->addDrawable(trajectory);
    osg::LineWidth* linewidth = new osg::LineWidth();
    linewidth->setWidth(4.0f);
    geode->getOrCreateStateSet()->setAttributeAndModes(linewidth, osg::StateAttribute::ON);

    //stipple for dashed lines:
    if (pattern > 1)
    {
      osg::LineStipple* linestipple = new osg::LineStipple;
      linestipple->setFactor(1);
      if (pattern == 2)
        linestipple->setPattern(0xf0f0);
      if (pattern == 3)
        linestipple->setPattern(0xff00);
      if (pattern == 4)
        linestipple->setPattern(0xf000);
      geode->getOrCreateStateSet()->setAttributeAndModes(linestipple, osg::StateAttribute::ON);
    }

    //Attach the trajectory to a switch node son of the localizedWorld node
    findNodeVisitor finder("localizedWorld");
    rootNode->accept(finder);
    std::vector<osg::Node*> node_list = finder.getNodeList();

    osg::Switch *swNode = new osg::Switch();
    swNode->setNewChildDefaultValue(true);
    swNode->setName("switch_trajectory");
    node_list[0]->asGroup()->addChild(swNode);



    const std::string SIMULATOR_DATA_PATH = std::string(getenv("HOME")) + "/.uwsim/data";
    osgDB::Registry::instance()->getDataFilePathList().push_back(std::string(UWSIM_ROOT_PATH) + std::string("/data/shaders"));
    static const char model_vertex[] = "default_scene.vert";
    static const char model_fragment[] = "default_scene.frag";

    osg::ref_ptr < osg::Program > program = osgOcean::ShaderManager::instance().createProgram("robot_shader",
                                                                                              model_vertex,
                                                                                              model_fragment, "", "");
    program->addBindAttribLocation("aTangent", 6);

    geode->getOrCreateStateSet()->setAttributeAndModes(program, osg::StateAttribute::ON);
    geode->getStateSet()->addUniform(new osg::Uniform("uOverlayMap", 1));
    geode->getStateSet()->addUniform(new osg::Uniform("uNormalMap", 2));
    swNode->addChild(geode);

    //Save LocalizedWorld inverted matrix
    LWMat=getWorldCoords(findRN("localizedWorld",rootNode));
    LWMat->invert(*LWMat);
  }
};

#endif /* TRAJECTORYVISUALIZATION_H_ */
