#ifndef VIEWBUILDER_H
#define VIEWBUILDER_H

#include <osgWidget/Util>
#include <osgOcean/OceanScene>

#include "osgOceanScene.h"
#include "HUDCamera.h"
#include "ROSInterface.h"
#include "SimulatedIAUV.h"
#include "ConfigXMLParser.h"
#include "SceneBuilder.h"

class ViewBuilder 
{
public:
	osg::ref_ptr<osgViewer::Viewer> viewer;
	boost::shared_ptr<osg::ArgumentParser> arguments;

public:
	ViewBuilder(ConfigFile &config, SceneBuilder *scene_builder);
	ViewBuilder(ConfigFile &config, SceneBuilder *scene_builder, int *argc, char **argv);
	ViewBuilder(ConfigFile &config, SceneBuilder *scene_builder, boost::shared_ptr<osg::ArgumentParser> args);

	osgViewer::View* getView() {return viewer.get();}
	osgViewer::Viewer* getViewer() {return viewer.get();}

	~ViewBuilder() {}

protected:
	bool init(ConfigFile &config, SceneBuilder *scene_builder);
};



#endif




