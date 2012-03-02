#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>
#include <osgGA/FlightManipulator>
#include <osgGA/TrackballManipulator>
#include <osgGA/NodeTrackerManipulator>
#include <osgGA/DriveManipulator>
#include <osgGA/StateSetManipulator>
#include <osgGA/GUIEventHandler>
#include <osg/Notify>
#include <osg/TextureCubeMap>
#include <osgDB/ReadFile>
#include <osgDB/WriteFile>
#include <osg/Shape>
#include <osg/ShapeDrawable>
#include <osg/PositionAttitudeTransform>
#include <osg/Program>
#include <osgText/Text>
#include <osg/CullFace>
#include <osg/Fog>
#include <osgText/Font>
#include <osg/Switch>
#include <osg/Texture3D>

#include <osgWidget/Util>
#include <osgWidget/WindowManager>
#include <osgWidget/ViewerEventHandlers>
#include <osgWidget/Box>


#include <string>
#include <vector>
#include <boost/shared_ptr.hpp>

#include <osgOcean/Version>
#include <osgOcean/OceanScene>
#include <osgOcean/FFTOceanSurface>
#include <osgOcean/SiltEffect>
#include <osgOcean/ShaderManager>

#include "osgOceanScene.h"
#include "SkyDome.h"
#include "TextHUD.h"
#include "EventHandler.h"
#include "HUDCamera.h"
#include "ConfigXMLParser.h"
#include "VirtualRangeSensor.h"

#include "ROSInterface.h"
#include "UWSimUtils.h"

using namespace std;

#define USE_CUSTOM_SHADER

#include "BulletPhysics.h"


int main(int argc, char *argv[])
{
    osg::notify(osg::ALWAYS) << "UWSim; using osgOcean " << osgOceanGetVersion() << std::endl;

    osg::ArgumentParser arguments(&argc,argv);
    arguments.getApplicationUsage()->setApplicationName(arguments.getApplicationName());
    arguments.getApplicationUsage()->setDescription(arguments.getApplicationName()+" is using osgOcean.");
    arguments.getApplicationUsage()->setCommandLineUsage(arguments.getApplicationName()+" [options] ...");
    arguments.getApplicationUsage()->addCommandLineOption("--windx <x>","Wind X direction.");
    arguments.getApplicationUsage()->addCommandLineOption("--windy <y>","Wind Y direction.");
    arguments.getApplicationUsage()->addCommandLineOption("--windSpeed <speed>","Wind speed.");
    arguments.getApplicationUsage()->addCommandLineOption("--isNotChoppy","Set the waves not choppy.");
    arguments.getApplicationUsage()->addCommandLineOption("--choppyFactor <factor>","How choppy the waves are.");
    arguments.getApplicationUsage()->addCommandLineOption("--crestFoamHeight <height>","How high the waves need to be before foam forms on the crest.");
    arguments.getApplicationUsage()->addCommandLineOption("--oceanSurfaceHeight <z>","Z position of the ocean surface in world coordinates.");
    arguments.getApplicationUsage()->addCommandLineOption("--disableShaders","Disable use of shaders for the whole application. Also disables most visual effects as they depend on shaders.");
    arguments.getApplicationUsage()->addCommandLineOption("--disableTextures","Disable use of textures by default. Can be toggled with the 't' key.");
    arguments.getApplicationUsage()->addCommandLineOption("--resw <width>","Set the viewer width resolution");
    arguments.getApplicationUsage()->addCommandLineOption("--resh <height>","Set the viewer height resolution");
    arguments.getApplicationUsage()->addCommandLineOption("--freeMotion","Sets the main camera to move freely");
    arguments.getApplicationUsage()->addCommandLineOption("--configfile","Indicate config file location (default: data/scenes/cirs.xml). The rest of the options override the values defined in this file.");
    arguments.getApplicationUsage()->addCommandLineOption("--v","Be verbose. (OSG notify level NOTICE)");
    arguments.getApplicationUsage()->addCommandLineOption("--vv","Be much verbose. (OSG notify level DEBUG)");

    unsigned int helpType = 0;
    if ((helpType = arguments.readHelpType()))
    {
        arguments.getApplicationUsage()->write(std::cout, helpType);
        return 1;
    }

    // report any errors if they have occurred when parsing the program arguments.
    if (arguments.errors())
    {
        arguments.writeErrorMessages(std::cout);


        return 1;
    }

    string configfile=std::string(SIMULATOR_DATA_PATH)+"/scenes/cirs.xml";
    while( arguments.read("--configfile",configfile));
    ConfigFile  config(configfile);

    float windx = config.windx, windy = config.windy;
    while (arguments.read("--windx", windx));
    while (arguments.read("--windy", windy));
    osg::Vec2f windDirection(windx, windy);

    float reswidth = config.resw, resheight = config.resh;
    while (arguments.read("--resw", reswidth));
    while (arguments.read("--resh", resheight));

    float windSpeed = config.windSpeed;
    while (arguments.read("--windSpeed", windSpeed));

    float depth = config.depth;
    //while (arguments.read("--depth", depth));

    float reflectionDamping = config.reflectionDamping;
    while (arguments.read("--reflectionDamping", reflectionDamping));

    float scale = config.waveScale;
    while (arguments.read("--waveScale", scale ) );

    bool isChoppy = not config.isNotChoppy;
    while (arguments.read("--isNotChoppy")) isChoppy = false;

    float choppyFactor = config.choppyFactor;
    while (arguments.read("--choppyFactor", choppyFactor));
    choppyFactor = -choppyFactor;

    float crestFoamHeight = config.crestFoamHeight;
    while (arguments.read("--crestFoamHeight", crestFoamHeight));

    double oceanSurfaceHeight = config.oceanSurfaceHeight;
    while (arguments.read("--oceanSurfaceHeight", oceanSurfaceHeight));

    bool disableShaders = config.disableShaders;
    if (arguments.read("--disableShaders")) disableShaders = true;

    bool disableTextures = false;
    if (arguments.read("--disableTextures")) disableTextures = true;

    bool freeMotion = config.freeMotion;
    if (arguments.read("--freeMotion")) {
    	freeMotion = true;
    }

    //Default notify level
    osg::setNotifyLevel(osg::FATAL);
    if (arguments.read("--v")) osg::setNotifyLevel(osg::NOTICE);
    if (arguments.read("--vv")) osg::setNotifyLevel(osg::DEBUG_FP);


    // any option left unread are converted into errors to write out later.
    arguments.reportRemainingOptionsAsUnrecognized();

    // report any errors if they have occurred when parsing the program arguments.
    if (arguments.errors())
    {
        arguments.writeErrorMessages(std::cout);
        return 1;
    }


    //Initialize viewer
    osgViewer::Viewer viewer;
    viewer.addEventHandler( new osgViewer::StatsHandler );
    osg::ref_ptr<TextHUD> hud = new TextHUD;
    osgOcean::ShaderManager::instance().enableShaders(!disableShaders);


    //Initialize ocean scene.
    osg::ref_ptr<osgOceanScene> scene = new osgOceanScene(config.offsetp, config.offsetr, windDirection, windSpeed, depth, reflectionDamping, scale, isChoppy, choppyFactor, crestFoamHeight);

    if (disableShaders)
    {
        // If shaders disabled, disable all special effects that depend on shaders.
        scene->getOceanScene()->enableDistortion(false);
        scene->getOceanScene()->enableGlare(false);
        scene->getOceanScene()->enableUnderwaterDOF(false);

        // These are only implemented in the shader, with no fixed-pipeline equivalent
        scene->getOceanScene()->enableUnderwaterScattering(false);
        // For these two, we might be able to use projective texturing so it would
        // work on the fixed pipeline?
        scene->getOceanScene()->enableReflections(false);
        scene->getOceanScene()->enableRefractions(false);
        scene->getOceanScene()->enableGodRays(false);  // Could be done in fixed pipeline?
        scene->getOceanScene()->enableSilt(false);     // Could be done in fixed pipeline?
    }

    scene->getOceanScene()->setOceanSurfaceHeight(oceanSurfaceHeight);
    viewer.addEventHandler(scene->getOceanSceneEventHandler());
    viewer.addEventHandler(scene->getOceanSurface()->getEventHandler());
    scene->getOceanScene()->setUnderwaterFog(config.fogDensity,  osg::Vec4f(config.fogColor[0],config.fogColor[1],config.fogColor[2],1) );
    scene->getOceanScene()->setUnderwaterDiffuse( osg::Vec4f(config.color[0],config.color[1],config.color[2],1) );
    scene->getOceanScene()->setUnderwaterAttenuation( osg::Vec3f(config.attenuation[0], config.attenuation[1], config.attenuation[2]) );
   
    //Add config file iauv
    std::vector<boost::shared_ptr<SimulatedIAUV> > iauvFile;
    int nvehicle=config.vehicles.size();
    for (int i=0; i<nvehicle; i++) {
      Vehicle vehicle=config.vehicles.front();
      boost::shared_ptr<SimulatedIAUV> siauv(new SimulatedIAUV(scene.get(),vehicle));
      iauvFile.push_back(siauv);
      config.vehicles.pop_front();

      scene->addObject(iauvFile[i]->baseTransform);

      siauv->setVehiclePosition(vehicle.position[0],vehicle.position[1],vehicle.position[2],vehicle.orientation[0],vehicle.orientation[1],vehicle.orientation[2]);
       
      if(vehicle.jointValues.size() && siauv->urdf!=NULL){
        siauv->urdf->setJointPosition(vehicle.jointValues);
      }  
    }

    //Add objects added in config file.
    while(config.objects.size()>0){
	Object auxObject= config.objects.front();

    	osg::Matrixd wMb_m;
	wMb_m.makeRotate(osg::Quat(auxObject.orientation[0],osg::Vec3d(1,0,0),auxObject.orientation[1],osg::Vec3d(0,1,0), auxObject.orientation[2],osg::Vec3d(0,0,1) ));
	wMb_m.setTrans(auxObject.position[0],auxObject.position[1],auxObject.position[2]);

	osg::ref_ptr<osg::MatrixTransform> wMb=new osg::MatrixTransform(wMb_m);
	scene->addObject(wMb, auxObject.file, &auxObject);
	wMb->setName(auxObject.name);

	config.objects.pop_front();
    }


    //Set main camera position, lookAt, and other params  
    if (freeMotion) {	
	osg::ref_ptr<osgGA::TrackballManipulator> tb = new osgGA::TrackballManipulator;
	tb->setHomePosition( osg::Vec3f(config.camPosition[0],config.camPosition[1],config.camPosition[2]), osg::Vec3f(config.camLookAt[0],config.camLookAt[1],config.camLookAt[2]), osg::Vec3f(0,0,1) );
	viewer.setCameraManipulator( tb );
    } else {	//Main camera tracks an object
        findRoutedNode findRN(config.vehicleToTrack);
        findRN.find(scene->getScene());

	osg::ref_ptr<osg::Node> first=findRN.getFirst();
	if (first.valid()) {
		//target to track found
		osg::ref_ptr<osg::Node> emptyNode= new osg::Node;
		osg::ref_ptr<osgGA::NodeTrackerManipulator> ntm = new osgGA::NodeTrackerManipulator;
	 	first->asGroup()->addChild(emptyNode);
		ntm->setTrackNode(emptyNode);
		ntm->setTrackerMode(osgGA::NodeTrackerManipulator::NODE_CENTER);
		ntm->setHomePosition(osg::Vec3d(config.camPosition[0],config.camPosition[1],config.camPosition[2]),osg::Vec3d(config.camLookAt[0],config.camLookAt[1],config.camLookAt[2]), osg::Vec3d(0,0,1));
		viewer.setCameraManipulator( ntm );
	} else {
		//Target object not found, free camera
		osg::ref_ptr<osgGA::TrackballManipulator> tb = new osgGA::TrackballManipulator;
		tb->setHomePosition( osg::Vec3f(config.camPosition[0],config.camPosition[1],config.camPosition[2]), osg::Vec3f(config.camLookAt[0],config.camLookAt[1],config.camLookAt[2]), osg::Vec3f(0,0,1) );
		viewer.setCameraManipulator( tb );
	}
    }
    viewer.addEventHandler( new osgViewer::HelpHandler );
    viewer.getCamera()->setName("MainCamera");
    //Main camera projection parameters: view angle (fov), aspect ratio, fustrum near and far
    if (config.camNear!=-1 && config.camFar!=-1) {
	OSG_INFO << "Setting custom near/far planes to " << config.camNear << " " << config.camFar << std::endl;
    	viewer.getCamera()->setComputeNearFarMode(osgUtil::CullVisitor::DO_NOT_COMPUTE_NEAR_FAR);
	viewer.getCamera()->setProjectionMatrixAsPerspective(config.camFov, config.camAspectRatio, config.camNear, config.camFar);
    } else {
	OSG_INFO << "Setting custom near/far planes to auto" << std::endl;
    	viewer.getCamera()->setProjectionMatrixAsPerspective(config.camFov, config.camAspectRatio, 1, 10);
    }

    //Set-up the scene graph and main loop
    osg::ref_ptr<osg::Group> root = new osg::Group;
    root->addChild(scene->getScene());

 //   iauv->lightSource->addChild(scene->getScene());	//Add vehicles light sources to the scene. Check if can be added to the .osg file.
 //   root->addChild( iauv->lightSource );

    OSG_INFO << "Setting main viewer" << std::endl;
    viewer.setSceneData( root );

    OSG_INFO << "Setting vehicle virtual cameras" << std::endl;
    for (int j=0; j<nvehicle ;j++){
      for (unsigned int i=0; i<iauvFile[j]->getNumCams(); i++) {
	    iauvFile[j]->camview[i].textureCamera->addChild( scene->getScene() );
	    root->addChild( iauvFile[j]->camview[i].textureCamera );
      }
    }

    ros::init(argc,argv,"UWSim");

    OSG_INFO << "Setting interfaces with external software..." << std::endl;
    std::vector<boost::shared_ptr<HUDCamera> > realcams;
    std::vector<boost::shared_ptr<ROSInterface> > ROSInterfaces;
    while(config.ROSInterfaces.size()>0){
      ROSInterfaceInfo rosInterface = config.ROSInterfaces.front();
     
      boost::shared_ptr<ROSInterface> iface; 
      if(rosInterface.type==ROSInterfaceInfo::ROSOdomToPAT)
	iface=boost::shared_ptr<ROSOdomToPAT>(new ROSOdomToPAT(root,rosInterface.topic,rosInterface.targetName));

      if(rosInterface.type==ROSInterfaceInfo::ROSTwistToPAT)
	iface=boost::shared_ptr<ROSTwistToPAT>(new ROSTwistToPAT(root,rosInterface.topic,rosInterface.targetName));
      
      if(rosInterface.type==ROSInterfaceInfo::PATToROSOdom)
	iface=boost::shared_ptr<PATToROSOdom>(new PATToROSOdom(root,rosInterface.targetName,rosInterface.topic,rosInterface.rate));

      if(rosInterface.type==ROSInterfaceInfo::ROSJointStateToArm || rosInterface.type==ROSInterfaceInfo::ArmToROSJointState) {
	//Find corresponding SimulatedIAUV Object
	for (int j=0; j<nvehicle ;j++){
		if (iauvFile[j]->name==rosInterface.targetName) {
		   if (rosInterface.type==ROSInterfaceInfo::ROSJointStateToArm)
			iface=boost::shared_ptr<ROSJointStateToArm>(new ROSJointStateToArm(rosInterface.topic,iauvFile[j]));
		   else
			iface=boost::shared_ptr<ArmToROSJointState>(new ArmToROSJointState(iauvFile[j].get(),rosInterface.topic,rosInterface.rate));
		}
        }
      }

      if(rosInterface.type==ROSInterfaceInfo::VirtualCameraToROSImage) 
	//Find corresponding VirtualCamera Object on all the vehicles
	for (int j=0; j<nvehicle ;j++)
	  for (unsigned int c=0; c<iauvFile[j]->getNumCams(); c++) 
		if (iauvFile[j]->camview[c].name==rosInterface.targetName) 
		   iface=boost::shared_ptr<VirtualCameraToROSImage>(new VirtualCameraToROSImage(&(iauvFile[j]->camview[c]),rosInterface.topic, rosInterface.infoTopic, rosInterface.rate));

      if(rosInterface.type==ROSInterfaceInfo::ROSImageToHUD) {
	boost::shared_ptr<HUDCamera> realcam(new HUDCamera(rosInterface.w,rosInterface.h, rosInterface.posx, rosInterface.posy, rosInterface.scale));
	iface=boost::shared_ptr<ROSImageToHUDCamera>(new ROSImageToHUDCamera(rosInterface.topic, rosInterface.infoTopic, realcam.get()));
        realcams.push_back(realcam);
      }

      if(rosInterface.type==ROSInterfaceInfo::RangeSensorToROSRange)
	//Find corresponding VirtualRangeSensor Object on all the vehicles
	for (int j=0; j<nvehicle ;j++)
	  for (unsigned int c=0; c<iauvFile[j]->getNumRangeSensors(); c++)
		if (iauvFile[j]->range_sensors[c].name==rosInterface.targetName)
		   iface=boost::shared_ptr<RangeSensorToROSRange>(new RangeSensorToROSRange(&(iauvFile[j]->range_sensors[c]),rosInterface.topic, rosInterface.rate));

      ROSInterfaces.push_back(iface);
      config.ROSInterfaces.pop_front();
    }


    //root->addChild(physics.debugDrawer.getSceneGraph());

    OSG_INFO << "Starting window manager..." << std::endl;
    osg::ref_ptr<osgWidget::WindowManager> wm = new osgWidget::WindowManager(
		&viewer,
		reswidth,
		resheight,
		0xF0000000,
		0
    );

    OSG_INFO << "Setting widgets..." << std::endl;
    wm->setPointerFocusMode(osgWidget::WindowManager::PFM_SLOPPY);
    std::vector<osg::ref_ptr<osgWidget::Window> > camWidgets;
    int dispx=0;
    int ncamwidgets=0;
    for (int j=0; j<nvehicle ;j++){
      for (unsigned int i=0; i<iauvFile[j]->getNumCams(); i++) {
	    camWidgets.push_back(iauvFile[j]->camview[i].getWidgetWindow());
	    camWidgets[ncamwidgets]->setX(dispx);
	    camWidgets[ncamwidgets]->setY(0);
	    dispx+=iauvFile[j]->camview[i].width+20;
	    wm->addChild(camWidgets[ncamwidgets]);
	    camWidgets[ncamwidgets]->hide();
	    viewer.addEventHandler( new SceneEventHandler(camWidgets[ncamwidgets], hud.get(), scene) );
	    ncamwidgets++;
      }
    }

    for (unsigned int i=0; i<realcams.size(); i++) {
	    wm->addChild(realcams[i]->getWidgetWindow());
    }
    
    //viewer.addEventHandler( new SceneEventHandler(NULL, hud.get(), scene ) );

    OSG_INFO << "Creating application..." << std::endl;

    viewer.setUpViewInWindow(
        50,
        50,
        static_cast<int>(wm->getWidth()),
        static_cast<int>(wm->getHeight())
    );

    osg::ref_ptr<osg::Group>  appgroup  = new osg::Group();
    osg::ref_ptr<osg::Camera> appcamera = wm->createParentOrthoCamera();

    appgroup->addChild(appcamera);

    if(root) appgroup->addChild(root);

    viewer.addEventHandler(new osgWidget::MouseHandler(wm));
    viewer.addEventHandler(new osgWidget::KeyboardHandler(wm));
    viewer.addEventHandler(new osgWidget::ResizeHandler(wm, appcamera));
    viewer.addEventHandler(new osgWidget::CameraSwitchHandler(wm, appcamera));
    viewer.addEventHandler(new osgViewer::StatsHandler());
    viewer.addEventHandler(new osgViewer::WindowSizeHandler());
    viewer.addEventHandler(new osgGA::StateSetManipulator(
        viewer.getCamera()->getOrCreateStateSet()
    ));

    //If --disableTextures, force disabling textures
    if (disableTextures) {
	    for (unsigned int ii=0; ii<4; ii++) {
		#if !defined(OSG_GLES1_AVAILABLE) && !defined(OSG_GLES2_AVAILABLE)
		    viewer.getCamera()->getOrCreateStateSet()->setTextureMode( ii, GL_TEXTURE_1D, osg::StateAttribute::OVERRIDE|osg::StateAttribute::OFF );
		#endif
		viewer.getCamera()->getOrCreateStateSet()->setTextureMode( ii, GL_TEXTURE_2D, osg::StateAttribute::OVERRIDE|osg::StateAttribute::OFF );
		viewer.getCamera()->getOrCreateStateSet()->setTextureMode( ii, GL_TEXTURE_3D, osg::StateAttribute::OVERRIDE|osg::StateAttribute::OFF );
		viewer.getCamera()->getOrCreateStateSet()->setTextureMode( ii, GL_TEXTURE_RECTANGLE, osg::StateAttribute::OVERRIDE|osg::StateAttribute::OFF );
		viewer.getCamera()->getOrCreateStateSet()->setTextureMode( ii, GL_TEXTURE_CUBE_MAP, osg::StateAttribute::OVERRIDE|osg::StateAttribute::OFF);
	    }
    }

    wm->resizeAllWindows();


    viewer.setSceneData(appgroup);

    viewer.realize();
        viewer.frame();
#ifdef BUILD_BULLET_PHYSICS
    double prevSimTime = 0.;
#endif

     typedef osgViewer::Viewer::Windows Windows;
     Windows windows;
     viewer.getWindows(windows);
     windows[0]->setWindowName("UWSim");

    while( !viewer.done() && ros::ok())
    {
	ros::spinOnce();

#ifdef BUILD_BULLET_PHYSICS
        const double currSimTime = viewer.getFrameStamp()->getSimulationTime();
        double elapsed( currSimTime - prevSimTime );
        if( viewer.getFrameStamp()->getFrameNumber() < 3 )
            elapsed = 1./60.;
  	//physics.stepSimulation(elapsed, 8, btScalar(1.)/btScalar(200.) );
	physics.stepSimulation(elapsed, 4, elapsed/4. );
        prevSimTime = currSimTime;
#endif
        viewer.frame();

    }
    if (ros::ok()) ros::shutdown();

/*
    std::cerr << "Stopping ROSInterfaces" << std::endl;
    for (int i=0; i<ROSInterfaces.size(); i++)
	ROSInterfaces[i].reset();
    std::cerr << "Release scene graph" << std::endl;
    root.release();
    std::cerr << "Stopping Vehicles" << std::endl;
    for (int i=0; i<iauvFile.size(); i++)
	iauvFile[i].reset();
*/
    OSG_INFO << "Finished" << std::endl;

    return 0;
}
