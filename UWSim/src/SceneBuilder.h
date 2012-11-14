#ifndef SCENEBUILDER_H
#define SCENEBUILDER_H

#include <osgWidget/Util>
#include <osgOcean/OceanScene>

#include "osgOceanScene.h"
#include "HUDCamera.h"
#include "ROSInterface.h"
#include "SimulatedIAUV.h"
#include "ConfigXMLParser.h"

class SceneBuilder 
{
public:
	boost::shared_ptr<osg::ArgumentParser> arguments;
	osg::ref_ptr<osgOceanScene> scene;
	std::vector<boost::shared_ptr<SimulatedIAUV> > iauvFile;
        std::vector<osg::ref_ptr<osg::Node> > objects;

	osg::ref_ptr<osg::Group> root;

	std::vector<boost::shared_ptr<HUDCamera> > realcams;
	std::vector<boost::shared_ptr<ROSInterface> > ROSInterfaces;
	std::vector<osg::ref_ptr<osgWidget::Window> > camWidgets;


public:
	SceneBuilder();
	SceneBuilder(int *argc, char **argv);
	SceneBuilder(boost::shared_ptr<osg::ArgumentParser> args);

	bool loadScene(std::string xml_file);
	bool loadScene(ConfigFile config);

	osg::Group* getRoot() {return root.get();}
	osgOceanScene* getScene() {return scene.get();}

	~SceneBuilder();
};

#endif




