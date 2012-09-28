#include <QFileDialog>
#include <QStringList>
#include <QProcess>
#include <QFile>
#include <QProgressDialog>

#include <ROSInterface.h>
#include <osgPCDLoader.h>
#include "MainWindow.h"
#include "MosaicEventHandler.h"
#include "MosaicManipulator.h"

#include <osg/Matrixd>
#include <osg/ComputeBoundsVisitor>
#include <osgGA/TrackballManipulator>

#include <ros/package.h>



#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <auv_msgs/PerformInterventionStrategyAction.h>
#include <auv_msgs/SetInterventionConfig.h>
#include <auv_msgs/InterventionSpec.h>

#include <mar_perception/VirtualImage.h>

#include <visp/vpImage.h>
#include <visp/vpImageIo.h>
//#include <visp/vpDisplayX.h>
#include <visp/vpImageConvert.h>
#include <visp/vpPoint.h>
#include <visp/vpImagePoint.h>
#include <visp/vpPixelMeterConversion.h>

#include <kdl/frames.hpp>

#include <odat/training_data.h>
#include <odat_ros/conversions.h>
#include <object_detection/shape_processing.h>
#include <vision_msgs/TrainingData.h>
#include <vision_msgs/TrainDetector.h>

using namespace std;

static int dbRowCallback(void *map, int argc, char **argv, char **azColName)
{
	std::map<std::string,std::pair<std::string, std::string> > *local_map=(std::map<std::string,std::pair<std::string, std::string> >*)map;

	local_map->insert(std::pair<std::string, std::pair<std::string, std::string> >(argv[0], std::pair<std::string,std::string>(argv[1], argv[2])));
	return 0;
}

//TODO: Define an API for DB access that allows to later switch from sqlite to ROS database_interface
void MainWindow::updateDBHandsList() {
	map.clear();

	char *zErrMsg;
	int eres = sqlite3_exec(db, "SELECT * from hands", dbRowCallback, &map, &zErrMsg);
	if( eres!=SQLITE_OK ){
		std::cerr << "SQL error: " << zErrMsg << std::endl;;
		sqlite3_free(zErrMsg);
	}
}

bool MainWindow::deleteDBHand(std::string hand_name) {
	char *zErrMsg;
	int eres = sqlite3_exec(db, ("DELETE from hands where name='"+hand_name+"'").c_str(), 0, 0, &zErrMsg);
	if( eres!=SQLITE_OK ){
		std::cerr << "SQL error: " << zErrMsg << std::endl;;
		sqlite3_free(zErrMsg);
		return false;
	}
	updateDBHandsList();
	return true;
}

bool MainWindow::insertDBHand(std::string hand_name, std::string hand_package, std::string hand_path) {
	char *zErrMsg;
	int eres = sqlite3_exec(db, ("INSERT into hands (name, package, path) values ('"+hand_name+"','"+hand_package+"','"+hand_path+"')").c_str(), 0, 0, &zErrMsg);
	if( eres!=SQLITE_OK ){
		std::cerr << "SQL error: " << zErrMsg << std::endl;;
		sqlite3_free(zErrMsg);
		return false;
	}
	updateDBHandsList();
	return true;
}


MainWindow::MainWindow(boost::shared_ptr<osg::ArgumentParser> arguments): arguments_(arguments) {

	ui.setupUi(this);

	openedFileName = new QLabel("No file opened. Default scene loaded");
	oldManipulator=NULL;
	//marker=NULL;
	//grasp=NULL;
	//database=new database_interface::PostgresqlDatabase("arkadia.act.uji.es", "5432", "postgres", "****", "handsBD");
	//TODO: Allow the user to specify a custom sqlite DB file, instead of the default QtUWSim/handsDB
	int rc = sqlite3_open((ros::package::getPath("QtUWSim")+std::string("/handsDB")).c_str(), &db);
	if( rc ) {
		ROS_ERROR("Can't open database: %s", sqlite3_errmsg(db));
		sqlite3_close(db);
		exit(0);
	} else {
		updateDBHandsList();
	}

	string configfile=std::string(SIMULATOR_DATA_PATH)+"/scenes/init.xml";
	while( arguments->read("--configfile",configfile));
	ConfigFile config(configfile);
	sceneBuilder=boost::shared_ptr<SceneBuilder>(new SceneBuilder(arguments));
	sceneBuilder->loadScene(config);
	viewBuilder=boost::shared_ptr<ViewBuilder>(new ViewBuilder(config, sceneBuilder.get(), arguments));
	scene=sceneBuilder->getScene();
	root=sceneBuilder->getRoot();


	prevSimTime = ros::Time::now();
	viewWidget=boost::shared_ptr<ViewerWidget>(new ViewerWidget(viewBuilder->getView()));
	viewWidget->setGeometry(200,200,800,600);
	setCentralWidget(viewWidget.get());


	//offsetp.push_back(config.offsetp[0]);
	//offsetp.push_back(config.offsetp[1]);
	//offsetp.push_back(config.offsetp[2]);
	//offsetr.push_back(config.offsetr[0]);
	//offsetr.push_back(config.offsetr[1]);
	//offsetr.push_back(config.offsetr[2]);


	/*dockHands=new QDockWidget("Hands",this);
	gridHands=new QGridLayout;
	hands=new QListWidget(dockHands);
	gridHands->addWidget(hands,0,0);
	buttonsHandsWidget=new QWidget();
	gridButtonsHands=new QGridLayout;
	newHandButton=new QPushButton("New", this);
	deleteHandButton=new QPushButton("Delete", this);
	gridButtonsHands->addWidget(newHandButton,0,0);
	gridButtonsHands->addWidget(deleteHandButton,0,1);
	buttonsHandsWidget->setLayout(gridButtonsHands);
	gridHands->addWidget(buttonsHandsWidget,1,0);
	handsWidget=new QWidget();
	handsWidget->setLayout(gridHands);
	dockHands->setWidget(handsWidget);
	addDockWidget(Qt::LeftDockWidgetArea, dockHands);*/


	/*dockConfigurations=new QDockWidget("Shapes",this);
	gridConfigurations=new QGridLayout;
	configurations=new QListWidget(dockConfigurations);
	gridConfigurations->addWidget(configurations,0,0);
	buttonsConfigurationsWidget=new QWidget();
	gridButtonsConfigurations=new QGridLayout;
	newConfigurationButton=new QPushButton("New",this);
	deleteConfigurationButton=new QPushButton("Delete",this);
	gridButtonsConfigurations->addWidget(newConfigurationButton,0,0);
	gridButtonsConfigurations->addWidget(deleteConfigurationButton,0,1);
	buttonsConfigurationsWidget->setLayout(gridButtonsConfigurations);
	gridConfigurations->addWidget(buttonsConfigurationsWidget,1,0);
	configurationsWidget=new QWidget();
	configurationsWidget->setLayout(gridConfigurations);
	dockConfigurations->setWidget(configurationsWidget);
	addDockWidget(Qt::LeftDockWidgetArea, dockConfigurations);*/

	/////////////////////////////////////////////////////////////////////////////
	//Section to fill the "Tool->Scene configuration->OceanState" data////////////////////////
	ui.windXDoubleSpin->setValue(config.windx);
	ui.windYDoubleSpin->setValue(config.windy);
	ui.windSpeedDoubleSpin->setValue(config.windSpeed);
	ui.depthDoubleSpin->setValue(config.depth);
	ui.reflectionDoubleSpin->setValue(config.reflectionDamping);
	ui.waveScaleDoubleSpin->setValue(config.waveScale);
	ui.isNotChoppyDoubleSpin->setValue(config.isNotChoppy);
	ui.choppyFactorDoubleSpin->setValue(config.choppyFactor);
	ui.crestFoamDoubleSpin->setValue(config.crestFoamHeight);
	ui.surfaceHeightDoubleSpin->setValue(config.oceanSurfaceHeight);
	ui.fogDensityDoubleSpin->setValue(config.fogDensity);
	ui.fogColorRDoubleSpin->setValue(config.fogColor[0]); ui.fogColorGDoubleSpin->setValue(config.fogColor[1]); ui.fogColorBDoubleSpin->setValue(config.fogColor[2]);
	ui.oceanColorRDoubleSpin->setValue(config.color[0]); ui.oceanColorGDoubleSpin->setValue(config.color[1]); ui.oceanColorBDoubleSpin->setValue(config.color[2]);
	ui.attenuationRDoubleSpin->setValue(config.attenuation[0]); ui.attenuationGDoubleSpin->setValue(config.attenuation[1]); ui.attenuationBDoubleSpin->setValue(config.attenuation[2]);

	//Section to fill the "Tool->Scene configuration->SimParams" data////////////////////////
	ui.disableShadersDoubleSpin->setValue(config.disableShaders);
	ui.resolutionWDoubleSpin->setValue(config.resw);
	ui.resolutionHDoubleSpin->setValue(config.resh);
	ui.offsetRxDoubleSpin->setValue(config.offsetr[0]);ui.offsetRyDoubleSpin->setValue(config.offsetr[1]);ui.offsetRzDoubleSpin->setValue(config.offsetr[2]);
	ui.offsetPxDoubleSpin->setValue(config.offsetp[0]);ui.offsetPyDoubleSpin->setValue(config.offsetp[1]);ui.offsetPzDoubleSpin->setValue(config.offsetp[2]);
	//Section to fill the "Tool->Scene configuration->Camera" data////////////////////////
	ui.freeMotionDoubleSpin->setValue(config.freeMotion);
	ui.fovDoubleSpin->setValue(config.camFov);
	ui.aspectRatioDoubleSpin->setValue(config.camAspectRatio);
	ui.nearDoubleSpin->setValue(config.camNear);
	ui.farDoubleSpin->setValue(config.camFar);
	ui.cameraPositionXDoubleSpin->setValue(config.camPosition[0]); ui.cameraPositionYDoubleSpin->setValue(config.camPosition[1]); ui.cameraPositionZDoubleSpin->setValue(config.camPosition[2]);
	ui.cameraLookatXDoubleSpin->setValue(config.camLookAt[0]); ui.cameraLookatYDoubleSpin->setValue(config.camLookAt[1]); ui.cameraLookatZDoubleSpin->setValue(config.camLookAt[2]);
	/////////////////////////////////////////////////////////////////////////////
	/////////////////////////////////////////////////////////////////////////////


	ui.dockIntervention3D->hide();
	ui.dockIntervention2D->hide();
	ui.dockSceneConfig->hide();

	frame_manager=FrameManager::instance();
	frame_manager->setFixedFrame("world");

	connect(&timer, SIGNAL(timeout()), this, SLOT(rosSpin()));
	timer.start(100);
	connect(ui.actionXML, SIGNAL(triggered()), this, SLOT(loadXML()));
	connect(ui.actionCreateMosaic, SIGNAL(triggered()), this, SLOT(createMosaic()));
	connect(ui.actionLoadMosaic, SIGNAL(triggered()), this, SLOT(loadMosaic()));

	connect(ui.actionIntervention2D, SIGNAL(triggered()), this, SLOT(showIntervention2DPanel()));
	connect(ui.newIntervention2D, SIGNAL(clicked()), this, SLOT(newIntervention2D()));
	connect(ui.deleteIntervention2D, SIGNAL(clicked()), this, SLOT(deleteIntervention2D()));
	connect(ui.graspIntervention2D, SIGNAL(clicked()), this, SLOT(graspIntervention2D()));
	connect(ui.Intervention2DList, SIGNAL(itemSelectionChanged()), this, SLOT(selectedIntervention2D()));
	connect(ui.exportIntervention2D, SIGNAL(clicked()), this, SLOT(exportIntervention2D()));

	connect(ui.actionIntervention3D, SIGNAL(triggered()), this, SLOT(graspSpecification()));
	connect(ui.newHandButton, SIGNAL(clicked()), this, SLOT(newHand()));
	connect(ui.hands, SIGNAL(itemSelectionChanged()), this, SLOT(handChanged())); //TODO: why not itemDoubleClick()?
	connect(ui.deleteHandButton, SIGNAL(clicked()),this, SLOT(deleteHand()));
	connect(ui.actionAbout, SIGNAL(triggered()), this, SLOT(about()));
	createStatusBar();
	connect(ui.actionScene_Configuration, SIGNAL(triggered()), this, SLOT(sceneConfiguration()));
	connect(ui.LoadSceneButton, SIGNAL(clicked()),this,SLOT(loadNewScene()));

}


MainWindow::~MainWindow(){}


void MainWindow::loadXML(){
	//FIXME: dialog is not deleted at the end -> memory leak. This code is full of them :(
	//		 USE SMART POINTERS or take care of pointers!
	QFileDialog *dialog= new QFileDialog(this, "Load XML", ".","XML Files (*.xml)");
	if(dialog->exec()){
		if (viewBuilder)
			viewBuilder.reset();
		if (sceneBuilder)
			sceneBuilder.reset();

		QProgressDialog progressDialog("Loading the XML file...", "Cancel process", 0, 100, this);
		progressDialog.setWindowModality(Qt::WindowModal);
		progressDialog.setCancelButton(0);
		progressDialog.setValue(5);
		QApplication::processEvents();
		sleep(1.0);
		progressDialog.setValue(10);
		//		cout<<"UwsimProgressBarUpdate = 10 "<<endl;

		QStringList fichs=dialog->selectedFiles();
		std::string fichero = fichs[0].toUtf8().constData();
		ConfigFile config(fichero);

		progressDialog.setValue(25);
		sceneBuilder.reset(new SceneBuilder());
		sceneBuilder->loadScene(config);
		progressDialog.setValue(50);
		viewBuilder.reset(new ViewBuilder(config, sceneBuilder.get()));
		progressDialog.setValue(75);
		scene=sceneBuilder->getScene();
		root=sceneBuilder->getRoot();
		viewWidget->updateViewerWidget(viewBuilder->getView());
		progressDialog.setValue(100);
	}
	delete(dialog);
}


void MainWindow::createMosaic(){
	QStringList mosaic_file;
	QFileDialog *loadTextureDialog=new QFileDialog(this, "Load texture Mosaic", ".", "Image Files (*)");
	if(loadTextureDialog->exec()){
		QStringList textura=loadTextureDialog->selectedFiles();
		QFileDialog *LoadHeightDialog=new QFileDialog(this, "Load height Mosaic", ".", "ImageFiles(*)");
		int val=LoadHeightDialog->exec();
		QStringList height;
		if(val)
			height=LoadHeightDialog->selectedFiles();
		QFileDialog *SaveMosaicDialog=new QFileDialog(this, "Save osg Mosaic", ".", "OSG Files (*.osg)");
		SaveMosaicDialog->setAcceptMode(QFileDialog::AcceptSave);
		if(SaveMosaicDialog->exec()){
			SaveMosaicDialog->setDefaultSuffix("osg");
			mosaic_file=SaveMosaicDialog->selectedFiles();
			QString arg;
			QProcess *process;

			QProgressDialog progressDialog("Please, wait until the mosaic has been created...", "Cancel process", 0, 100, this);
			progressDialog.setWindowModality(Qt::WindowModal);
			progressDialog.setCancelButton(0);
			progressDialog.setValue(5);
			QApplication::processEvents();
			sleep(1.0);
			progressDialog.setValue(10);
			//			cout<<"UwsimProgressBarUpdate = 10 "<<endl;

			arg="gdal_translate ";
			arg.append(textura[0]);
			arg.append(" texture.tif");
			process->execute(arg);
			arg="gdaladdo -r average texture.tif 2 4 8 16 32";
			process->execute(arg);
			if(val){
				arg="gdal_translate ";
				arg.append(height[0]);
				arg.append(" height.tif");
				process->execute(arg);
				arg="gdaladdo -r average height.tif 2 4 8 16 32";
				process->execute(arg);
				arg="osgdem --xx 0.2 --yy 0.2 -t texture.tif --xx 0.02 --yy 0.02 -d height.tif -l 3 -v 1 -o ";
			}
			else{
				arg="osgdem --xx 0.08 --yy 0.08 -t texture.tif -l 3 -v 1 -o ";
			}

			progressDialog.setValue(25);

			image=new QImage("texture.tif");
			int ancho=image->size().rwidth();
			int alto=image->size().rheight();


			arg.append(mosaic_file[0]);
			process->execute(arg);
			arg="rm texture.tif";
			process->execute(arg);
			if(val){
				arg="rm height.tif";
				process->execute(arg);
			}
			viewBuilder.reset();
			sceneBuilder.reset();

			osgDB::Registry::instance()->getDataFilePathList().push_back(std::string(mosaic_file[0].toUtf8().constData()));
			ConfigFile config(std::string (ros::package::getPath("QtUWSim"))+"/mosaics/mosaic.xml");
			config.objects.front().file=mosaic_file[0].toUtf8().constData();
			cout<<"Nombre del fichero: "<<config.objects.front().file<<endl;
			config.objects.front().position[0]=-((ancho*0.01)/2);
			config.objects.front().position[1]=-((alto*0.01)/2);
			offsetp.clear();
			offsetr.clear();
			offsetp.push_back(config.offsetp[0]);
			offsetp.push_back(config.offsetp[1]);
			offsetp.push_back(config.offsetp[2]);
			offsetr.push_back(config.offsetr[0]);
			offsetr.push_back(config.offsetr[1]);
			offsetr.push_back(config.offsetr[2]);

			progressDialog.setValue(40);

			sceneBuilder=boost::shared_ptr<SceneBuilder>(new SceneBuilder());
			progressDialog.setValue(60);

			sceneBuilder->loadScene(config);
			viewBuilder=boost::shared_ptr<ViewBuilder>(new ViewBuilder(config, sceneBuilder.get()));
			scene=sceneBuilder->getScene();
			root=sceneBuilder->getRoot();
			planar_grasp_spec_.clear();
			planar_grasp_spec_.push_back(boost::shared_ptr<PlanarGraspSpec>(new PlanarGraspSpec("spec",root)));
			viewBuilder->getView()->addEventHandler(new MosaicEventHandler(planar_grasp_spec_[0].get(), this, viewWidget.get()));
			viewWidget->updateViewerWidget(viewBuilder->getView());
			progressDialog.setValue(80);
			MosaicManipulator *tb=new MosaicManipulator;
			osg::Vec3d eye, center, up;
			oldManipulator=viewBuilder->getView()->getCameraManipulator();
			oldManipulator->getHomePosition(eye, center, up);
			tb->setHomePosition(eye, center, up);
			oldManipulator=viewBuilder->getView()->getCameraManipulator();
			viewBuilder->getView()->setCameraManipulator(tb);
			progressDialog.setValue(100);
			delete(process);
		}
		delete(SaveMosaicDialog);
		delete(LoadHeightDialog);
	}
	delete(loadTextureDialog);
}


void MainWindow::loadMosaic(){
	QStringList mosaic_file;
	QFileDialog *loadOsgDialog=new QFileDialog(this, "Load osg Mosaic", ".", "OSG Files (*.osg *.ive *.pcd)");
	if(loadOsgDialog->exec()){
		mosaic_file=loadOsgDialog->selectedFiles();
		updateStatusBar(mosaic_file[0]);
		if (viewBuilder)
			viewBuilder.reset();
		if (sceneBuilder)
			sceneBuilder.reset();

		osg::Node *mosaic_node;
		boost::shared_ptr<osgPCDLoader<pcl::PointXYZRGB> > pcd_geode;
		if (osgDB::getFileExtension(mosaic_file[0].toStdString())=="pcd") {
			//Load PCD
			pcd_geode.reset(new osgPCDLoader<pcl::PointXYZRGB>(mosaic_file[0].toStdString()));
			mosaic_node=pcd_geode->getGeode();
		} else {
			//Load osg / ive
			mosaic_node=osgDB::readNodeFile(mosaic_file[0].toStdString());
		}
		if (mosaic_node!=NULL) {
			root=new osg::Group();
			root->addChild(mosaic_node);
			mosaic_viewer_=osg::ref_ptr<osgViewer::Viewer>(new osgViewer::Viewer());
			mosaic_viewer_->setSceneData(root);
			if (osgDB::getFileExtension(mosaic_file[0].toStdString())=="pcd")
				mosaic_viewer_->setCameraManipulator(new osgGA::TrackballManipulator());
			else
				mosaic_viewer_->setCameraManipulator(new MosaicManipulator());

			//get object center and bounds
			osg::ComputeBoundsVisitor cbVisitor;
			mosaic_node->accept(cbVisitor);
			osg::BoundingBox bs = cbVisitor.getBoundingBox();
			osg::BoundingSphere bsphere;
			bsphere.expandBy(bs);
			//Get the minimum bbox dimension, and set the camera looking along that direction
			osg::Vec3d geom_up(1,0,0);
			if ((bs.yMax()-bs.yMin()) < (bs.xMax()-bs.xMin())) {
				geom_up=osg::Vec3d(0,1,0);
			    if ((bs.zMax()-bs.zMin()) < (bs.yMax()-bs.yMin())) geom_up=osg::Vec3d(0,0,1);
			} else if ((bs.zMax()-bs.zMin()) < (bs.xMax()-bs.xMin())) geom_up=osg::Vec3d(0,0,1);
			//Get the maximum bbox dimension, and set the camera up vector along that direction
			osg::Vec3d geom_longest(1,0,0);
			if ((bs.yMax()-bs.yMin()) > (bs.xMax()-bs.xMin())) {
				geom_longest=osg::Vec3d(0,1,0);
			    if ((bs.zMax()-bs.zMin()) > (bs.yMax()-bs.yMin())) geom_longest=osg::Vec3d(0,0,1);
			} else if ((bs.zMax()-bs.zMin()) > (bs.xMax()-bs.xMin())) geom_longest=osg::Vec3d(0,0,1);
			mosaic_viewer_->getCameraManipulator()->setHomePosition(bs.center()+osg::Vec3d(geom_up[0]*1*bsphere.radius(),geom_up[1]*1*bsphere.radius(),geom_up[2]*1*bsphere.radius()), bs.center(), geom_longest);
			mosaic_viewer_->getCameraManipulator()->home(0);
			planar_grasp_spec_.clear();
			//view->addEventHandler(new MosaicEventHandler(planar_grasp_spec_[0].get(), this, viewWidget));
			viewWidget->updateViewerWidget(mosaic_viewer_);
			switches=new osg::Switch();
			root->addChild(switches);
		}
	}
	delete(loadOsgDialog);
}


void MainWindow::graspSpecification(){
	//if(!database->isConnected()){
	//	std::cerr<<"Database failed to connect"<<endl;
	//}
	//else{
	for(int i=ui.hands->count()-1; i>=0 ; i--){
		ui.hands->removeItemWidget(ui.hands->takeItem(i));
	}
	/*for(int i=configurations->count()-1; i>=0; i--){
			configurations->removeItemWidget(configurations->takeItem(i));
		}*/
	ui.dockIntervention2D->hide();
	ui.dockSceneConfig->hide();
	ui.dockIntervention3D->show();

	//if(!database->getList(handsDB)){
	//	std::cerr<<"Failed to get list of hands"<<endl;
	//}

	//for(size_t i=0; i<handsDB.size(); i++){
	//	ui.hands->addItem(handsDB[i]->name_hand.data().c_str());
	//}
	std::map<std::string,std::pair<std::string, std::string> >::iterator it;
	for(it=map.begin(); it!=map.end(); it++){
		ui.hands->addItem(it->first.c_str());
	}
	//}
	//ompl_ros_interface::OmplRos *ompl_ros=new ompl_ros_interface::OmplRos();
	//ompl_ros->run();    
}

void MainWindow::showIntervention2DPanel(){
	ui.dockIntervention3D->hide();
	ui.dockSceneConfig->hide();
	ui.dockIntervention2D->show();
}

void MainWindow::newIntervention2D(){
	bool ok;
	QString interventionName=QInputDialog::getText(this, "Name", "Name", QLineEdit::Normal, "",&ok);
	if (ok) {
		boost::shared_ptr<PlanarGraspSpec> spec(new PlanarGraspSpec(interventionName.toStdString(), switches));
		planar_grasp_spec_.push_back(spec);
		osg::Vec3d eye, center, up;
		mosaic_viewer_->getCamera()->getViewMatrixAsLookAt(eye, center, up);
		spec->setTemplateScale(eye[2]/2);
		eye[2]=0.01;
		spec->setTemplateOrigin(eye);

		ui.Intervention2DList->addItem(interventionName.toStdString().c_str());
		widget_to_spec_.insert(std::pair<QListWidgetItem*, int>(ui.Intervention2DList->item(ui.Intervention2DList->count()-1), planar_grasp_spec_.size()-1));
		ui.Intervention2DList->setCurrentRow(ui.Intervention2DList->count()-1);
		switches->setAllChildrenOff();
		switches->setChildValue(planar_grasp_spec_[widget_to_spec_[ui.Intervention2DList->currentItem()]]->t_transform,true);
		ui.deleteIntervention2D->setEnabled(true);




	}
}

void MainWindow::deleteIntervention2D(){
	QListWidgetItem *item=ui.Intervention2DList->currentItem();
	if (item) {
		cout<<"Borrar: "<<widget_to_spec_[item]<<endl;
		int currentRow=ui.Intervention2DList->currentRow();

		widget_to_spec_.erase(item);
		std::vector<boost::shared_ptr<PlanarGraspSpec> >::iterator it=planar_grasp_spec_.begin();
		for(; it<planar_grasp_spec_.end(); it++) {
			if (item->text().toStdString() == (*it)->getName()) {
				planar_grasp_spec_.erase(it);
			}
		}
		delete item;
		for(int i=currentRow; i<ui.Intervention2DList->size().rheight();i++){
					widget_to_spec_[ui.Intervention2DList->item(i)]-=1;
		}
		item=ui.Intervention2DList->currentItem();
		switches->setAllChildrenOff();
		if(item){
			switches->setChildValue(planar_grasp_spec_[widget_to_spec_[item]]->t_transform,true);
			ui.graspIntervention2D->setEnabled(!planar_grasp_spec_[widget_to_spec_[item]]->haveGrasp());
			ui.exportIntervention2D->setEnabled(planar_grasp_spec_[widget_to_spec_[item]]->haveGrasp());
		}
		else{
			ui.deleteIntervention2D->setEnabled(false);
			ui.graspIntervention2D->setEnabled(false);
			ui.exportIntervention2D->setEnabled(false);
		}
	}
}

void MainWindow::graspIntervention2D(){
	//TODO: Compute a grasp, allow the user to adjust it
	QListWidgetItem *item=ui.Intervention2DList->currentItem();
	planar_grasp_spec_[widget_to_spec_[item]]->createGraspDragger();
	ui.exportIntervention2D->setEnabled(true);
	ui.graspIntervention2D->setEnabled(false);
}

void MainWindow::selectedIntervention2D(){
	QListWidgetItem *item=ui.Intervention2DList->currentItem();
	std::cerr << "Called selectedIntervention2D " << widget_to_spec_[item] << std::endl;
	if (item) {
		//Unselect all
		for (int i=0; i<planar_grasp_spec_.size(); i++) planar_grasp_spec_[i]->setUnselected();

		//Select the new one and update info
	    planar_grasp_spec_[widget_to_spec_[item]]->setSelected();
		std::stringstream ss_position, ss_scale;
		ss_position << planar_grasp_spec_[widget_to_spec_[item]]->getTemplateOrigin();
		ui.labelPositionValue->setText(ss_position.str().c_str());
		ss_scale << planar_grasp_spec_[widget_to_spec_[item]]->getTemplateScale();
		ui.labelScaleValue->setText(ss_scale.str().c_str());

		switches->setAllChildrenOff();
		switches->setChildValue(planar_grasp_spec_[widget_to_spec_[item]]->t_transform,true);
		ui.graspIntervention2D->setEnabled(!planar_grasp_spec_[widget_to_spec_[item]]->haveGrasp());
		ui.exportIntervention2D->setEnabled(planar_grasp_spec_[widget_to_spec_[item]]->haveGrasp());

	}
}

void MainWindow::exportIntervention2D(){

	QListWidgetItem *item=ui.Intervention2DList->currentItem();
	if(item){
		QFileDialog *dialog=new QFileDialog(this, "Save Target Template", ".", "Image (*.png)");
		dialog->setAcceptMode(QFileDialog::AcceptSave);
		if(dialog->exec()){
			QFileDialog *dialog_txt=new QFileDialog(this, "Save Points Data", ".", "Text (*.txt)");
				dialog_txt->setAcceptMode(QFileDialog::AcceptSave);
				if(dialog_txt->exec()){
					dialog->setDefaultSuffix("png");
					QStringList target_file=dialog->selectedFiles();
					dialog_txt->setDefaultSuffix("txt");
					QStringList points_file=dialog_txt->selectedFiles();



					osg::Vec3d center,center_aux, scale;
					center_aux=planar_grasp_spec_[widget_to_spec_[item]]->getTemplateCenter();
					center=planar_grasp_spec_[widget_to_spec_[item]]->getTemplateOrigin();
					center[0]+=center_aux[0];
					center[1]-=center_aux[2];

					osg::Vec3 br, bl, tr, tl, g1, g2, br_m, bl_m, tr_m, tl_m, g1_m, g2_m;

					osg::MatrixList worldMatrices=((osgManipulator::CustomTabPlaneTrackballDragger*)planar_grasp_spec_[widget_to_spec_[item]]->t_dragger_)->_tabPlaneDragger->getCorners()->getWorldMatrices();
					for(osg::MatrixList::iterator itr=worldMatrices.begin();
							itr !=worldMatrices.end(); itr++)
					{
						osg::Matrix& matrix=*itr;
						bl_m=((osgManipulator::CustomTabPlaneTrackballDragger*)planar_grasp_spec_[widget_to_spec_[item]]->t_dragger_)->_tabPlaneDragger->getCorners()->getBottomLeftHandleNode()->getBound().center() * matrix;
						br_m=((osgManipulator::CustomTabPlaneTrackballDragger*)planar_grasp_spec_[widget_to_spec_[item]]->t_dragger_)->_tabPlaneDragger->getCorners()->getBottomRightHandleNode()->getBound().center() * matrix;
						tl_m=((osgManipulator::CustomTabPlaneTrackballDragger*)planar_grasp_spec_[widget_to_spec_[item]]->t_dragger_)->_tabPlaneDragger->getCorners()->getTopLeftHandleNode()->getBound().center() * matrix;
						tr_m=((osgManipulator::CustomTabPlaneTrackballDragger*)planar_grasp_spec_[widget_to_spec_[item]]->t_dragger_)->_tabPlaneDragger->getCorners()->getTopRightHandleNode()->getBound().center() * matrix;

					}

					worldMatrices=planar_grasp_spec_[widget_to_spec_[item]]->g_dragger_->getWorldMatrices();
					for(osg::MatrixList::iterator itr=worldMatrices.begin();
							itr !=worldMatrices.end(); itr++)
					{
						osg::Matrix& matrix=*itr;
						g1_m=planar_grasp_spec_[widget_to_spec_[item]]->g_dragger_->tdragger1_->getBound().center()* matrix;
						g2_m=planar_grasp_spec_[widget_to_spec_[item]]->g_dragger_->tdragger2_->getBound().center()* matrix;
					}


					float maxX=max(max(max(max(max(br_m[0],bl_m[0]),tr_m[0]),tl_m[0]),g1_m[0]),g2_m[0]);
					float minX=min(min(min(min(min(br_m[0],bl_m[0]),tr_m[0]),tl_m[0]),g1_m[0]),g2_m[0]);
					float maxY=max(max(max(max(max(br_m[1],bl_m[1]),tr_m[1]),tl_m[1]),g1_m[1]),g2_m[1]);
					float minY=min(min(min(min(min(br_m[1],bl_m[1]),tr_m[1]),tl_m[1]),g1_m[1]),g2_m[1]);

					//Render to texture
					osg::Vec3d eye,cent, up;
					mosaic_viewer_->getCameraManipulator()->getHomePosition(eye,cent,up);
					osg::Matrix matView=mosaic_viewer_->getCameraManipulator()->getMatrix();
					mosaic_viewer_->getCameraManipulator()->setHomePosition(osg::Vec3d(center[0],center[1],2*std::max(std::max(center[0]-minX,maxX-center[0]),std::max(center[1]-minY,maxY-center[1]))),center,osg::Vec3d(1,0,0));
					mosaic_viewer_->getCameraManipulator()->home(0);
					switches->setChildValue(planar_grasp_spec_[widget_to_spec_[item]]->t_transform,false);

					osgViewer::ScreenCaptureHandler::ScreenCaptureHandler::WriteToFile *operation=new osgViewer::ScreenCaptureHandler::ScreenCaptureHandler::WriteToFile("/tmp/template_grasp","png");
					osgViewer::ScreenCaptureHandler *capture=new osgViewer::ScreenCaptureHandler(operation);
					capture->captureNextFrame(*mosaic_viewer_->getViewerBase());
					viewWidget->frame();
					delete(capture);


					worldMatrices=((osgManipulator::CustomTabPlaneTrackballDragger*)planar_grasp_spec_[widget_to_spec_[item]]->t_dragger_)->_tabPlaneDragger->getCorners()->getWorldMatrices();
					for(osg::MatrixList::iterator itr=worldMatrices.begin();
							itr !=worldMatrices.end(); itr++)
					{
						osg::Matrix& matrix=*itr;
						bl_m=((osgManipulator::CustomTabPlaneTrackballDragger*)planar_grasp_spec_[widget_to_spec_[item]]->t_dragger_)->_tabPlaneDragger->getCorners()->getBottomLeftHandleNode()->getBound().center() * matrix;
						br_m=((osgManipulator::CustomTabPlaneTrackballDragger*)planar_grasp_spec_[widget_to_spec_[item]]->t_dragger_)->_tabPlaneDragger->getCorners()->getBottomRightHandleNode()->getBound().center() * matrix;
						tl_m=((osgManipulator::CustomTabPlaneTrackballDragger*)planar_grasp_spec_[widget_to_spec_[item]]->t_dragger_)->_tabPlaneDragger->getCorners()->getTopLeftHandleNode()->getBound().center() * matrix;
						tr_m=((osgManipulator::CustomTabPlaneTrackballDragger*)planar_grasp_spec_[widget_to_spec_[item]]->t_dragger_)->_tabPlaneDragger->getCorners()->getTopRightHandleNode()->getBound().center() * matrix;

					}

					worldMatrices=planar_grasp_spec_[widget_to_spec_[item]]->g_dragger_->getWorldMatrices();
					for(osg::MatrixList::iterator itr=worldMatrices.begin();
							itr !=worldMatrices.end(); itr++)
					{
						osg::Matrix& matrix=*itr;
						g1_m=planar_grasp_spec_[widget_to_spec_[item]]->g_dragger_->tdragger1_->getBound().center()* matrix;
						g2_m=planar_grasp_spec_[widget_to_spec_[item]]->g_dragger_->tdragger2_->getBound().center()* matrix;
					}

					osg::Matrix win=mosaic_viewer_->getCamera()->getViewport()->computeWindowMatrix();
					osg::Matrix view=mosaic_viewer_->getCamera()->getViewMatrix();
					osg::Matrix proj=mosaic_viewer_->getCamera()->getProjectionMatrix();






					br=br_m*view*proj*win;
					bl=bl_m*view*proj*win;
					tr=tr_m*view*proj*win;
					tl=tl_m*view*proj*win;
					g1=g1_m*view*proj*win;
					g2=g2_m*view*proj*win;








					QString arg="cp /tmp/template_grasp_1_0.png ";
					arg.append(target_file[0]);
					QProcess *process;
					process->execute(arg);
					arg="rm /tmp/template_grasp_1_0.png";
					process->execute(arg);
					delete(process);
					QImage *image= new QImage(target_file[0]);
					int im_height=image->height();


					QFile file(points_file[0]);
					file.open(QIODevice::WriteOnly | QIODevice::Text);
					QTextStream out(&file);
					out<<br[0]<<","<<im_height-br[1]<<endl;
					out<<bl[0]<<","<<im_height-bl[1]<<endl;
					out<<tr[0]<<","<<im_height-tr[1]<<endl;
					out<<tl[0]<<","<<im_height-tl[1]<<endl;
					out<<g1[0]<<","<<im_height-g1[1]<<endl;
					out<<g2[0]<<","<<im_height-g2[1]<<endl;
					file.close();
					switches->setChildValue(planar_grasp_spec_[widget_to_spec_[item]]->t_transform,true);
					mosaic_viewer_->getCameraManipulator()->setHomePosition(eye,cent,up);
					mosaic_viewer_->getCameraManipulator()->setByMatrix(matView);





					ros::NodeHandle nh;

					vpImage<vpRGBa> Ic;
					vpImageIo::read(Ic,target_file[0].toStdString());
					//vpDisplayX window(Ic);

					vpImagePoint clicks[3];
					std::string uji_intervention_config_topic;
					uji_intervention_config_topic="Nombre del servicio";
					ros::ServiceClient config_client_=nh.serviceClient<auv_msgs::SetInterventionConfig>(uji_intervention_config_topic);
					ros::Publisher training_data_pub_;
					training_data_pub_ = nh.advertise<vision_msgs::TrainingData>("/trainer_node/training_data", 1);
					ROS_INFO("Call to %s", uji_intervention_config_topic.c_str());
					auv_msgs::SetInterventionConfig tracking_init_msg;
					tracking_init_msg.request.spec.id="intervencion_prueba";
					tracking_init_msg.request.spec.camera_frame=std::string(target_file[0].toStdString());
					tracking_init_msg.request.spec.roi.x_offset=bl[0];
					tracking_init_msg.request.spec.roi.y_offset=bl[1];
					tracking_init_msg.request.spec.roi.width=vpImagePoint::distance(vpImagePoint(tl[0],tl[1]),vpImagePoint(tr[0],tr[1]));
					tracking_init_msg.request.spec.roi.height=vpImagePoint::distance(vpImagePoint(bl[0],bl[1]),vpImagePoint(tl[0],tl[1]));
					tracking_init_msg.request.spec.roi_rotation=atan2(tr[1]-tl[1],tr[0]-tl[0]);


					//Compute tMg in pixels (grasp frame wrt to template frame)
					vpHomogeneousMatrix iMt_p(tracking_init_msg.request.spec.roi.x_offset, tracking_init_msg.request.spec.roi.y_offset, 0, 0, 0, tracking_init_msg.request.spec.roi_rotation);


					 vpHomogeneousMatrix iMg_p;
					 iMg_p=vpHomogeneousMatrix((g1[0]+g2[0])/2, g1[1]+g2[1]/2, 0, 0, 0, 0)*
							vpHomogeneousMatrix(0,0,0,M_PI,0,0)*
						  vpHomogeneousMatrix(0,0,0,0,0,-atan2(g1[0]-(g1[0]+g2[0])/2, g1[1]-(g1[1]+g2[1])/2));

					 vpHomogeneousMatrix tMg_p=iMt_p.inverse()*iMg_p;

					auv_msgs::InterventionStrategySpec strat_spec_msg;
					strat_spec_msg.preshape=strat_spec_msg.PRESHAPE_CYLINDRICAL_PRECISION;
					//strat_spec_msg.grasp_frame.translation.x=tMg_p[0][3]*TSIZE_X/tracking_init_msg.request.spec.roi.width;
					//strat_spec_msg.grasp_frame.translation.y=tMg_p[1][3]*TSIZE_Y/tracking_init_msg.request.spec.roi.height;
					strat_spec_msg.grasp_frame.translation.x=std::abs(tr_m[0]-tl_m[0]);
					strat_spec_msg.grasp_frame.translation.y=std::abs(tl_m[1]-bl_m[0]);
					strat_spec_msg.grasp_frame.translation.z=0;
					KDL::Rotation tRg(tMg_p[0][0], tMg_p[0][1], tMg_p[0][2], tMg_p[1][0], tMg_p[1][1], tMg_p[1][2],  tMg_p[2][0], tMg_p[2][1], tMg_p[2][2]);
					tRg.GetQuaternion(strat_spec_msg.grasp_frame.rotation.x, strat_spec_msg.grasp_frame.rotation.y, strat_spec_msg.grasp_frame.rotation.z, strat_spec_msg.grasp_frame.rotation.w);

					geometry_msgs::Point32 p1,p2;
					p1.x=g1[0]; p1.y=g1[1]; p1.z=0;
					p2.x=g2[0]; p2.y=g2[1]; p2.z=0;
					strat_spec_msg.p1=p1;
					strat_spec_msg.p2=p2;

					auv_msgs::InterventionTaskSpec task_spec_msg;
					task_spec_msg.task_type=task_spec_msg.TASK_RECOVERY;
					task_spec_msg.strategy_spec.push_back(strat_spec_msg);

					tracking_init_msg.request.spec.task_spec.push_back(task_spec_msg);
					if (config_client_.call(tracking_init_msg))
					{
						ROS_INFO("  Result: %d", tracking_init_msg.response.ok);
					}
					else
					{
						ROS_ERROR("Failed to call service %s", uji_intervention_config_topic.c_str());
					}


					ROS_INFO("Sending training message to detector...");
					std::vector<cv::Point> cv_polygon;
					cv_polygon.push_back(cv::Point(bl[0],bl[1]));
					cv_polygon.push_back(cv::Point(tl[0],tl[1]));
					cv_polygon.push_back(cv::Point(tr[0],tr[1]));
					cv_polygon.push_back(cv::Point(br[0],br[1]));

					odat::Pose2D polygon_pose;
					polygon_pose.x=tracking_init_msg.request.spec.roi.x_offset;
					polygon_pose.y=tracking_init_msg.request.spec.roi.y_offset;
					polygon_pose.theta=tracking_init_msg.request.spec.roi_rotation;
					ROS_INFO_STREAM("polygon pose: " << polygon_pose.x << " " << polygon_pose.y << " " << polygon_pose.theta);
					cv::Mat Imat;
					vpImageConvert::convert(Ic,Imat);

					odat::TrainingData training_data;
						    training_data.image = Imat;
						    training_data.mask.roi = object_detection::shape_processing::boundingRect(cv_polygon);
						    training_data.mask.mask = object_detection::shape_processing::minimalMask(cv_polygon);

						    training_data.image_pose = polygon_pose;

						    vision_msgs::TrainingData training_data_msg;
						    odat_ros::toMsg(training_data, training_data_msg);
						    training_data_msg.object_id = tracking_init_msg.request.spec.id;

						    training_data_pub_.publish(training_data_msg);
						    ROS_INFO("Training message published.");
						    cv::imwrite(ros::package::getPath("trident_uji_pkg")+"/training_images/training_image_" + tracking_init_msg.request.spec.id + ".png", Imat);




						ros::ServiceClient training_service_client_;
						training_service_client_ = nh.serviceClient<vision_msgs::TrainDetector>("/object_detector/train");
						if (training_service_client_.exists()) {
							ROS_INFO("Calling detector training service...");
							sensor_msgs::ImageConstPtr training_image_msg_;
							vision_msgs::TrainDetector training;
							training.request.object_id = tracking_init_msg.request.spec.id;

							assert(Imat.type() == CV_8UC3);
							cv_bridge::CvImage cv_image;
							cv_image.image = Imat;
							cv_image.encoding = sensor_msgs::image_encodings::BGR8;
							cv_image.toImageMsg(training.request.image_left);

							training.request.outline.points.resize(cv_polygon.size());
							for (size_t i = 0; i < cv_polygon.size(); ++i)
							{
								training.request.outline.points[i].x = cv_polygon[i].x;
								training.request.outline.points[i].y = cv_polygon[i].y;
								training.request.outline.points[i].z = 0;
							}
							geometry_msgs::Point32 grasp_point;
							training.request.outline.points.push_back(p1);
							training.request.outline.points.push_back(p2);
							training.request.image_pose.x=tracking_init_msg.request.spec.roi.x_offset;
							training.request.image_pose.y=tracking_init_msg.request.spec.roi.y_offset;
							training.request.image_pose.theta=tracking_init_msg.request.spec.roi_rotation;
							if (training_service_client_.call(training))
							{
								if (training.response.success == false)
								{
									ROS_ERROR("Training failed: %s", training.response.message.c_str());
								}
								else
								{
									ROS_INFO("Training succeeded: %s", training.response.message.c_str());
								}
							}
							else
							{
								ROS_ERROR("Training service call failed!");
							}
						} else {
							ROS_INFO("Training service not detected");
						}




				}
				delete(dialog_txt);
		}

		delete(dialog);
	}
}

void MainWindow::newHand(){
	bool ok;
	QString packageHand=QInputDialog::getText(this, "Select package ROS", "Package ROS", QLineEdit::Normal, "",&ok);
	if(ok && !packageHand.isEmpty()){
		std::string packagePath=ros::package::getPath(packageHand.toUtf8().constData());
		if(packagePath!="")
		{
			QFileDialog *dialog=new QFileDialog(this, "Path to new hand", packagePath.c_str(), "URDF Files (*.urdf)");
			if(dialog->exec()){
				QString pathHand=dialog->selectedFiles()[0];
				QString nameHand=QInputDialog::getText(this, "Insert the name of the hand", "Hand's name", QLineEdit::Normal, "",&ok);
				if(ok && !nameHand.isEmpty()){
					//int max=0;
					//for(size_t i=0; i<handsDB.size(); i++){
					//	if(max<handsDB[i]->hand_id.data())
					//		max=handsDB[i]->hand_id.data();
					//}
					//Hands newHand;
					//newHand.hand_id.data()=max+1;
					//newHand.name_hand.data()=nameHand.toUtf8().constData();
					//newHand.package_ros.data()=packageHand.toUtf8().constData();
					//newHand.path.data()=pathHand.split(packagePath.c_str())[1].toUtf8().constData();
					//if(!database->insertIntoDatabase(&newHand))
					//	std::cerr <<"Hand insertion failed"<<std::endl;
					//else{
					//	
					//	ui.hands->addItem(newHand.name_hand.data().c_str());
					//	if(!database->getList(handsDB)){
					//		std::cerr<<"Failed to get list of hands"<<endl;
					//	}
					//}
					if(!insertDBHand(nameHand.toUtf8().constData(), packageHand.toUtf8().constData(), pathHand.split(packagePath.c_str())[1].toUtf8().constData()))
						std::cerr <<"Hand insertion failed"<<std::endl;
					else{
						ui.hands->addItem(nameHand.toUtf8().constData()); //TODO: Call to a method updateUIHandList
					}
				}
			}
			delete(dialog);
		}
	}
	else{
		if(ok){
			QErrorMessage *error=new QErrorMessage();
			packageHand.prepend("couldn't find package ");
			error->showMessage(packageHand);
			delete(error);
			//if(!database->getList(handsDB)){
			//	std::cerr<<"Failed to get list of hands"<<endl;
			//}
		}
	}
}


void MainWindow::handChanged(){
	if(grasp)
		grasp.reset();
	grasp=boost::shared_ptr<GraspSpecification>(new GraspSpecification);
	if(marker){
		marker.reset();
	}
	if(joint_marker_cli){
		joint_marker_cli.reset();
	}
	if(ui.hands->currentRow()!=-1){
		//int cont=0;

		//while((handsDB[cont]->name_hand.data()!=ui.hands->item(ui.hands->currentRow())->text().toUtf8().constData())   &&  (cont < ui.hands->count())){
		//cont++;
		//}
		//if(cont< ui.hands->count()){
		//joint_marker_cli= new InteractiveMarkerDisplay("osg_im","/basic_controls/update", scene->localizedWorld, *(frame_manager->getTFClient()));
		//marker=new HandInteractiveMarker(handsDB[cont]->package_ros.data(),handsDB[cont]->path.data());
		//}
		joint_marker_cli= boost::shared_ptr<InteractiveMarkerDisplay>(new InteractiveMarkerDisplay("osg_im","/IM_JOINT_SERVER/update", scene->localizedWorld, *(frame_manager->getTFClient())));
		hand_marker_cli= boost::shared_ptr<InteractiveMarkerDisplay>(new InteractiveMarkerDisplay("osg_hand_im","/hand_pose_im/update", scene->localizedWorld, *(frame_manager->getTFClient())));
		marker=boost::shared_ptr<HandInteractiveMarker>(new HandInteractiveMarker(sceneBuilder,
				ui.hands->item(ui.hands->currentRow())->text().toUtf8().constData(),
				map[ui.hands->item(ui.hands->currentRow())->text().toUtf8().constData()].first,
				map[ui.hands->item(ui.hands->currentRow())->text().toUtf8().constData()].second));
	}
}


void MainWindow::deleteHand(){
	//int cont=0;
	//if(ui.hands->currentRow()!=-1){
	//while((handsDB[cont]->name_hand.data()!=ui.hands->item(ui.hands->currentRow())->text().toUtf8().constData())   &&  (cont < ui.hands->count())){
	//cont++;
	//}
	//if(cont< ui.hands->count()){
	//if(!database->deleteFromDatabase(handsDB[cont].get())){
	//std::cerr <<"Hand delete failed"<<std::endl;
	//}
	//int file=ui.hands->currentRow();
	//ui.hands->setCurrentRow(-1);
	//ui.hands->removeItemWidget(ui.hands->takeItem(file));
	//}
	//}
	if (deleteDBHand(ui.hands->item(ui.hands->currentRow())->text().toUtf8().constData())) {
		int file=ui.hands->currentRow();
		ui.hands->setCurrentRow(-1);
		ui.hands->removeItemWidget(ui.hands->takeItem(file));
	}

}


void MainWindow::rosSpin(){
	static tf::TransformBroadcaster br;

	if (marker && marker->uwsim_object && marker->uwsim_object->baseTransform) {
		//Publish the tf of the HandInteractiveMarker base frame
		geometry_msgs::Pose pose=marker->pose;

		tf::Transform transform;
		transform.setOrigin( tf::Vector3(pose.position.x,pose.position.y, pose.position.z) );
		transform.setRotation( tf::Quaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w) );
		br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", marker->uwsim_object->urdf->link[0]->getName())); //FIXME: multiple vehicles
	}

	ros::spinOnce();
	ros::Time currSimTime = ros::Time::now();
	ros::Duration elapsed= currSimTime - prevSimTime ;
	if(joint_marker_cli && hand_marker_cli){
		joint_marker_cli->update(elapsed.toSec(), elapsed.toSec());
		hand_marker_cli->update(elapsed.toSec(), elapsed.toSec());
	}
	prevSimTime=currSimTime;
}


void MainWindow::about(){
	QMessageBox::about(this, tr("About QtUWSim"),
			tr("<h2>QtUWSim</h2>"
					"<p>Software developed at <a href=\"http://www.irs.uji.es\">Interactive & Robotic Systems Lab</a></p>"
					"<p>Please, visit the <a href=\"http://www.irs.uji.es/uwsim\">official website</a> for news and the <a href=\"http://www.irs.uji.es/uwsim/wiki\">wiki</a> for help and info.</p>"));
}


void MainWindow::createStatusBar(){
	openedFileName->setMinimumSize(openedFileName->sizeHint());
	openedFileName->setAlignment(Qt::AlignCenter);
	ui.statusbar->addWidget(openedFileName);
}


void MainWindow::updateStatusBar(const QString name){
	openedFileName->setText(name);
	ui.statusbar->update();
}

void MainWindow::sceneConfiguration(){
	ui.dockIntervention3D->hide();
	ui.dockIntervention2D->hide();
	ui.dockSceneConfig->show();
}
