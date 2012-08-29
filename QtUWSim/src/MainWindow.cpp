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
	joint_marker_cli=NULL;
	hand_marker_cli=NULL;
	marker=NULL;
	grasp=NULL;
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

	string configfile=std::string(SIMULATOR_DATA_PATH)+"/scenes/cirs.xml";
	while( arguments->read("--configfile",configfile));
	ConfigFile config(configfile);
	sceneBuilder=boost::shared_ptr<SceneBuilder>(new SceneBuilder(arguments));
	sceneBuilder->loadScene(config);
	viewBuilder=boost::shared_ptr<ViewBuilder>(new ViewBuilder(config, sceneBuilder.get(), arguments));
	scene=sceneBuilder->getScene();
	root=sceneBuilder->getRoot();


	prevSimTime = ros::Time::now();
	viewWidget=new ViewerWidget(viewBuilder->getView());
	viewWidget->setGeometry(200,200,800,600);  
	setCentralWidget(viewWidget);


	offsetp.push_back(config.offsetp[0]);
	offsetp.push_back(config.offsetp[1]);
	offsetp.push_back(config.offsetp[2]);
	offsetr.push_back(config.offsetr[0]);
	offsetr.push_back(config.offsetr[1]);
	offsetr.push_back(config.offsetr[2]);


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
		//TODO: What is local offsetp and offsetr for? can't use those inside config?
		offsetp.clear();
		offsetr.clear();
		offsetp.push_back(config.offsetp[0]);
		offsetp.push_back(config.offsetp[1]);
		offsetp.push_back(config.offsetp[2]);
		offsetr.push_back(config.offsetr[0]);
		offsetr.push_back(config.offsetr[1]);
		offsetr.push_back(config.offsetr[2]);
		//sceneBuilder->stopROSInterfaces();
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
			viewBuilder->getView()->addEventHandler(new MosaicEventHandler(planar_grasp_spec_[0].get(), this, viewWidget));
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
		}	 
	}
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
		}
	}
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
		boost::shared_ptr<PlanarGraspSpec> spec(new PlanarGraspSpec(interventionName.toStdString(), root));
		planar_grasp_spec_.push_back(spec);
		osg::Vec3d eye, center, up;
		mosaic_viewer_->getCamera()->getViewMatrixAsLookAt(eye, center, up);
		spec->setTemplateScale(eye[2]/2);
		eye[2]=0.01;
		spec->setTemplateOrigin(eye);

		ui.Intervention2DList->addItem(interventionName.toStdString().c_str());
		widget_to_spec_.insert(std::pair<QListWidgetItem*, int>(ui.Intervention2DList->item(ui.Intervention2DList->count()-1), planar_grasp_spec_.size()-1));
	}
}

void MainWindow::deleteIntervention2D(){
	QListWidgetItem *item=ui.Intervention2DList->currentItem();
	if (item) {
		widget_to_spec_.erase(item);

		std::vector<boost::shared_ptr<PlanarGraspSpec> >::iterator it=planar_grasp_spec_.begin();
		for(; it<planar_grasp_spec_.end(); it++) {
			if (item->text().toStdString() == (*it)->getName()) {
				planar_grasp_spec_.erase(it);
			}
		}
		delete item;
	}
}

void MainWindow::graspIntervention2D(){
	//TODO: Compute a grasp, allow the user to adjust it
	QListWidgetItem *item=ui.Intervention2DList->currentItem();
	planar_grasp_spec_[widget_to_spec_[item]]->createGraspDragger();
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
		}
	}
	else{
		if(ok){
			QErrorMessage *error=new QErrorMessage();
			packageHand.prepend("couldn't find package ");
			error->showMessage(packageHand);
			//if(!database->getList(handsDB)){
			//	std::cerr<<"Failed to get list of hands"<<endl;
			//}
		}
	}
}


void MainWindow::handChanged(){
	if(grasp!=NULL)
		delete(grasp);
	grasp=new GraspSpecification();
	if(marker!=NULL){
		delete(marker);
		marker=NULL;
	}
	if(joint_marker_cli!=NULL){
		delete(joint_marker_cli);
		joint_marker_cli=NULL;
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
		joint_marker_cli= new InteractiveMarkerDisplay("osg_im","/IM_JOINT_SERVER/update", scene->localizedWorld, *(frame_manager->getTFClient()));
		hand_marker_cli= new InteractiveMarkerDisplay("osg_hand_im","/hand_pose_im/update", scene->localizedWorld, *(frame_manager->getTFClient()));
		marker=new HandInteractiveMarker(sceneBuilder,
				ui.hands->item(ui.hands->currentRow())->text().toUtf8().constData(),
				map[ui.hands->item(ui.hands->currentRow())->text().toUtf8().constData()].first,
				map[ui.hands->item(ui.hands->currentRow())->text().toUtf8().constData()].second);
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
	if(joint_marker_cli!=NULL && hand_marker_cli!=NULL){
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
