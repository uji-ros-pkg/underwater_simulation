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

#include <ros/ros.h>

#include <string>
#include <vector>
#include <stdlib.h>

#include <uwsim/ConfigXMLParser.h>
#include <uwsim/SceneBuilder.h>
#include <uwsim/ViewBuilder.h>
#include <uwsim/PhysicsBuilder.h>

#include <uwsim/UWSimUtils.h>

using namespace std;

//#include "BulletPhysics.h"

int main(int argc, char *argv[])
{
	//osg::notify(osg::ALWAYS) << "UWSim; using osgOcean " << osgOceanGetVersion() << std::endl;

	boost::shared_ptr<osg::ArgumentParser> arguments(new osg::ArgumentParser(&argc,argv));
	arguments->getApplicationUsage()->setApplicationName(arguments->getApplicationName());
	arguments->getApplicationUsage()->setDescription(arguments->getApplicationName()+" is using osgOcean.");
	arguments->getApplicationUsage()->setCommandLineUsage(arguments->getApplicationName()+" [options] ...");
	arguments->getApplicationUsage()->addCommandLineOption("--windx <x>","Wind X direction.");
	arguments->getApplicationUsage()->addCommandLineOption("--windy <y>","Wind Y direction.");
	arguments->getApplicationUsage()->addCommandLineOption("--windSpeed <speed>","Wind speed.");
	arguments->getApplicationUsage()->addCommandLineOption("--isNotChoppy","Set the waves not choppy.");
	arguments->getApplicationUsage()->addCommandLineOption("--choppyFactor <factor>","How choppy the waves are.");
	arguments->getApplicationUsage()->addCommandLineOption("--crestFoamHeight <height>","How high the waves need to be before foam forms on the crest.");
	arguments->getApplicationUsage()->addCommandLineOption("--oceanSurfaceHeight <z>","Z position of the ocean surface in world coordinates.");
	arguments->getApplicationUsage()->addCommandLineOption("--disableShaders","Disable use of shaders for the whole application. Also disables most visual effects as they depend on shaders.");
	arguments->getApplicationUsage()->addCommandLineOption("--disableTextures","Disable use of textures by default. Can be toggled with the 't' key.");
	arguments->getApplicationUsage()->addCommandLineOption("--resw <width>","Set the viewer width resolution");
	arguments->getApplicationUsage()->addCommandLineOption("--resh <height>","Set the viewer height resolution");
	arguments->getApplicationUsage()->addCommandLineOption("--freeMotion","Sets the main camera to move freely");
	arguments->getApplicationUsage()->addCommandLineOption("--configfile","Indicate config file location (default: data/scenes/cirs.xml). The rest of the options override the values defined in this file.");
	arguments->getApplicationUsage()->addCommandLineOption("--v","Be verbose. (OSG notify level NOTICE)");
	arguments->getApplicationUsage()->addCommandLineOption("--vv","Be much verbose. (OSG notify level DEBUG)");
	arguments->getApplicationUsage()->addCommandLineOption("--dataPath <path>","Search for models in this path, besides the default ones");

	unsigned int helpType = 0;
	if ((helpType = arguments->readHelpType()))
	{
		arguments->getApplicationUsage()->write(std::cout, helpType);
		return 1;
	}

	//Default notify level
	osg::setNotifyLevel(osg::FATAL);
	if (arguments->read("--v")) osg::setNotifyLevel(osg::NOTICE);
	if (arguments->read("--vv")) osg::setNotifyLevel(osg::DEBUG_FP);

	// report any errors if they have occurred when parsing the program arguments->
	if (arguments->errors())
	{
		arguments->writeErrorMessages(std::cout);

		return 1;
	}

	std::string dataPath("");
	while( arguments->read("--dataPath",dataPath));
	//Add current folder to path
	osgDB::Registry::instance()->getDataFilePathList().push_back(std::string("."));	
	//Add UWSim folders to path
        const std::string SIMULATOR_DATA_PATH = std::string(getenv("HOME")) + "/.uwsim/data";
	osgDB::Registry::instance()->getDataFilePathList().push_back(std::string(SIMULATOR_DATA_PATH));	

	osgDB::Registry::instance()->getDataFilePathList().push_back(std::string(UWSIM_ROOT_PATH));	
	osgDB::Registry::instance()->getDataFilePathList().push_back(std::string(UWSIM_ROOT_PATH) + "/data");	
	osgDB::Registry::instance()->getDataFilePathList().push_back(std::string(UWSIM_ROOT_PATH) + "/data/scenes");	

	//Add dataPath folder to path
	if (dataPath!=std::string("")) {
		osgDB::Registry::instance()->getDataFilePathList().push_back(dataPath);	
	}

	string configfile=std::string(UWSIM_ROOT_PATH)+"/scenes/cirs.xml";
	while( arguments->read("--configfile",configfile));
	ConfigFile config(configfile);

	ros::init(argc,argv,"UWSim");
	ros::start();

	SceneBuilder builder(arguments);
	builder.loadScene(config);

        PhysicsBuilder physicsBuilder;
        if(config.enablePhysics)
	  physicsBuilder.loadPhysics(&builder,config);

	ViewBuilder view(config, &builder, arguments);
	
	view.init();
	view.getViewer()->realize();
	view.getViewer()->frame();

	osgViewer::Viewer::Windows windows;
	view.getViewer()->getWindows(windows);
	windows[0]->setWindowName("UWSim");

	double prevSimTime = 0.;
	while( !view.getViewer()->done() && ros::ok())
	{
		ROSInterface::setROSTime(ros::Time::now());
		ros::spinOnce();

		if(config.enablePhysics){
                  const double currSimTime = view.getViewer()->getFrameStamp()->getSimulationTime();
                  double elapsed( currSimTime - prevSimTime );
                  if( view.getViewer()->getFrameStamp()->getFrameNumber() < 3 ) 
                    elapsed = 1./60.;
                  physicsBuilder.physics->stepSimulation(elapsed, 4, elapsed/4. );
                  prevSimTime = currSimTime;
		}                

		view.getViewer()->frame();
	}
	if (ros::ok()) ros::shutdown();

	ROS_INFO("Finished");

	return 0;
}
