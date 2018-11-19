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

#ifndef SCENEBUILDER_H
#define SCENEBUILDER_H

#include <osgWidget/Util>
#include <osgOcean/OceanScene>

#include "osgOceanScene.h"
#include "HUDCamera.h"
#include "ROSInterface.h"
#include "SimulatedIAUV.h"
#include "ConfigXMLParser.h"
#include <uwsim/CustomCommsChannel.h>
#include <uwsim/AcousticCommsChannel.h>
#include <uwsim/LedArray.h>

class SceneBuilder
{
public:
  std::shared_ptr<osg::ArgumentParser> arguments;
  osg::ref_ptr<osgOceanScene> scene;
  std::vector<std::shared_ptr<SimulatedIAUV> > iauvFile;
  std::vector<osg::ref_ptr<osg::Node> > objects;
  std::vector<osg::ref_ptr<osg::Node> > trajectories;

  osg::ref_ptr<osg::Group> root;

  std::vector<std::shared_ptr<HUDCamera> > realcams;
  std::vector<std::shared_ptr<ROSInterface> > ROSInterfaces;
  std::vector<osg::ref_ptr<osgWidget::Window> > camWidgets;
  std::vector<std::shared_ptr<uwsim::CustomCommsChannel>> customCommsChannels;
  std::vector<std::shared_ptr<uwsim::AcousticCommsChannel>> acousticCommsChannels;
  std::vector<std::shared_ptr<uwsim::LedArray>> ledArrays;

public:
  SceneBuilder();
  SceneBuilder(int *argc, char **argv);
  SceneBuilder(std::shared_ptr<osg::ArgumentParser> args);

  bool loadScene(std::string xml_file);
  bool loadScene(ConfigFile config);

  osg::Group* getRoot()
  {
    return root.get();
  }
  osgOceanScene* getScene()
  {
    return scene.get();
  }

  ~SceneBuilder();
};

#endif

