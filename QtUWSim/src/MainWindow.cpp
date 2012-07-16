#include <QFileDialog>
#include <QStringList>
#include <QProcess>
#include "ROSInterface.h"
#include "MainWindow.h"
#include "MosaicEventHandler.h"
#include <osg/Matrixd>
#include <ros/package.h>
#include <QFile>
#include <QProgressDialog>





using namespace std;

MainWindow::MainWindow(boost::shared_ptr<osg::ArgumentParser> arguments){

	ui.setupUi(this);
	
	openedFileName = new QLabel("No file opened. Default scene loaded");
	oldManipulator=NULL;
	marker_cli=NULL;
	marker=NULL;
	spec=NULL;
	grasp=NULL;
	database=new database_interface::PostgresqlDatabase("arkadia.act.uji.es", "5432", "postgres", "irslab2012", "handsBD");

	string configfile=std::string(SIMULATOR_DATA_PATH)+"/scenes/cirs.xml";
	while( arguments->read("--configfile",configfile));
	ConfigFile config(configfile);
	sceneBuilder=new SceneBuilder(arguments);
	sceneBuilder->loadScene(config);
	viewBuilder=new ViewBuilder(config, sceneBuilder, arguments);
	view=viewBuilder->getView();
	scene=sceneBuilder->getScene();
	root=sceneBuilder->getRoot();


	prevSimTime =view->getFrameStamp()->getSimulationTime();
	viewWidget=new ViewerWidget(view);
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


	ui.dockHand->hide();
	//dockConfigurations->hide();*/

	//FIXME: Set base_link and IM topic from arguments or XML
	frame_manager=FrameManager::instance();
	frame_manager->setFixedFrame("/base_link");

	connect(&timer, SIGNAL(timeout()), this, SLOT(rosSpin()));
	timer.start(100);
	connect(ui.actionXML, SIGNAL(triggered()), this, SLOT(loadXML()));
	connect(ui.actionCreateMosaic, SIGNAL(triggered()), this, SLOT(createMosaic()));
	connect(ui.actionLoadMosaic, SIGNAL(triggered()), this, SLOT(loadMosaic()));
	connect(ui.actionGrasp, SIGNAL(triggered()), this, SLOT(graspSpecification()));
	connect(ui.newHandButton, SIGNAL(clicked()), this, SLOT(newHand()));
	connect(ui.hands, SIGNAL(itemSelectionChanged()), this, SLOT(handChanged()));
	connect(ui.deleteHandButton, SIGNAL(clicked()),this, SLOT(deleteHand()));
	connect(ui.actionAbout, SIGNAL(triggered()), this, SLOT(about()));
	createStatusBar();
	connect(ui.pathButton, SIGNAL(clicked()),this,SLOT(newPath()));
}


MainWindow::~MainWindow(){}


void MainWindow::loadXML(){
	QFileDialog *dialog= new QFileDialog(this, "Load XML", ".","XML Files (*.xml)");
	if(dialog->exec()){
		delete viewBuilder;
		delete sceneBuilder;

		//TODO: Progress bar
		QStringList fichs=dialog->selectedFiles();
		std::string fichero = fichs[0].toUtf8().constData();
		ConfigFile config(fichero);
		offsetp.clear();
		offsetr.clear();
		offsetp.push_back(config.offsetp[0]);
		offsetp.push_back(config.offsetp[1]);
		offsetp.push_back(config.offsetp[2]);
		offsetr.push_back(config.offsetr[0]);
		offsetr.push_back(config.offsetr[1]);
		offsetr.push_back(config.offsetr[2]);
		//sceneBuilder->stopROSInterfaces();
		sceneBuilder=new SceneBuilder();
		sceneBuilder->loadScene(config);
		viewBuilder=new ViewBuilder(config, sceneBuilder);
		view=viewBuilder->getView();
		scene=sceneBuilder->getScene();
		root=sceneBuilder->getRoot();
		viewWidget->updateViewerWidget(view);
	}
}


void MainWindow::createMosaic(){
	QStringList mosaic_file;
	QFileDialog *dialog=new QFileDialog(this, "Load texture Mosaic", ".", "Image Files (*)");
	if(dialog->exec()){
		QStringList textura=dialog->selectedFiles();
		QFileDialog *dialog2=new QFileDialog(this, "Load height Mosaic", ".", "ImageFiles(*)");
		int val=dialog2->exec();
		QStringList height;
		if(val)
			height=dialog2->selectedFiles();
		QFileDialog *dialog3=new QFileDialog(this, "Save osg Mosaic", ".", "OSG Files (*.osg)");
		dialog3->setAcceptMode(QFileDialog::AcceptSave);
		if(dialog3->exec()){
			dialog3->setDefaultSuffix("osg");
			mosaic_file=dialog3->selectedFiles();
			QString arg;
			QProcess *process;

			QProgressDialog progress("Creating the mosaic...", "Cancel process", 0, 100, this);
			progress.setWindowModality(Qt::WindowModal);
			progress.setLabelText("Please, wait until the mosaic has been created.");
			progress.setValue(5);
			cout<<"UwsimProgressBarUpdate = 5 "<<endl;
			QApplication::processEvents();
			sleep(1.0);
			progress.setValue(10);
			cout<<"UwsimProgressBarUpdate = 10 "<<endl;

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

			progress.setValue(25);
			cout<<"UwsimProgressBarUpdate = 25 "<<endl;

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
			delete viewBuilder;
			delete(sceneBuilder);

			osgDB::Registry::instance()->getDataFilePathList().push_back(std::string(mosaic_file[0].toUtf8().constData()));
			ConfigFile config(std::string (ros::package::getPath("QtUWSim"))+"/mosaics/mosaic.xml");
			config.objects.front().file=mosaic_file[0].toUtf8().constData();
			cout<<"Nombre del Fichero "<<config.objects.front().file<<endl;
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

			progress.setValue(50);
			cout<<"UwsimProgressBarUpdate = 50 "<<endl;

			sceneBuilder=new SceneBuilder();
			progress.setValue(75);
			cout<<"UwsimProgressBarUpdate = 75 "<<endl;

			sceneBuilder->loadScene(config);
			viewBuilder=new ViewBuilder(config, sceneBuilder);
			view=viewBuilder->getView();
			scene=sceneBuilder->getScene();
			root=sceneBuilder->getRoot();
			if (spec!= NULL) delete spec;
			spec=new PlanarGraspSpec(root);
			view->addEventHandler(new MosaicEventHandler(spec, this, viewWidget));
			viewWidget->updateViewerWidget(view);
			MosaicManipulator *tb=new MosaicManipulator;
			osg::Vec3d eye, center, up;
			oldManipulator=view->getCameraManipulator();
			oldManipulator->getHomePosition(eye, center, up);
			tb->setHomePosition(eye, center, up);
			oldManipulator=view->getCameraManipulator();
			view->setCameraManipulator(tb);
			progress.setValue(100);
			cout<<"UwsimProgressBarUpdate = 100 "<<endl;
		}	 
	}
}


void MainWindow::loadMosaic(){
	QStringList mosaic_file;
	QFileDialog *dialog1=new QFileDialog(this, "Load osg Mosaic", ".", "OSG Files (*.osg)");
	if(dialog1->exec()){
		mosaic_file=dialog1->selectedFiles();
		updateStatusBar(mosaic_file[0]);
		delete viewBuilder;
		delete(sceneBuilder);
		osgDB::Registry::instance()->getDataFilePathList().push_back(std::string(mosaic_file[0].toUtf8().constData()));
		ConfigFile config(std::string (ros::package::getPath("QtUWSim"))+"/mosaics/mosaic.xml");
		config.objects.front().file=mosaic_file[0].toUtf8().constData();
		offsetp.clear();
		offsetr.clear();
		offsetp.push_back(config.offsetp[0]);
		offsetp.push_back(config.offsetp[1]);
		offsetp.push_back(config.offsetp[2]);
		offsetr.push_back(config.offsetr[0]);
		offsetr.push_back(config.offsetr[1]);
		offsetr.push_back(config.offsetr[2]);

		sceneBuilder=new SceneBuilder();
		sceneBuilder->loadScene(config);
		viewBuilder=new ViewBuilder(config, sceneBuilder);
		view=viewBuilder->getView();
		scene=sceneBuilder->getScene();
		root=sceneBuilder->getRoot();
		spec=new PlanarGraspSpec(root);
		view->addEventHandler(new MosaicEventHandler(spec, this, viewWidget));
		viewWidget->updateViewerWidget(view);
		MosaicManipulator *tb=new MosaicManipulator;
		oldManipulator=view->getCameraManipulator();
		osg::Vec3d eye, center, up;
		oldManipulator->getHomePosition(eye, center, up);
		tb->setHomePosition(eye, center, up);
		view->setCameraManipulator(tb);
	}
}


void MainWindow::graspSpecification(){
	if(!database->isConnected()){
		std::cerr<<"Database failed to connect"<<endl;
	}
	else{
		for(int i=ui.hands->count()-1; i>=0 ; i--){
			ui.hands->removeItemWidget(ui.hands->takeItem(i));
		}
		/*for(int i=configurations->count()-1; i>=0; i--){
			configurations->removeItemWidget(configurations->takeItem(i));
		}*/
		ui.dockHand->show();
		//dockConfigurations->show();


		if(!database->getList(handsDB)){
			std::cerr<<"Failed to get list of hands"<<endl;
		}

		for(size_t i=0; i<handsDB.size(); i++){
			ui.hands->addItem(handsDB[i]->name_hand.data().c_str());
		}
	}
	//ompl_ros_interface::OmplRos *ompl_ros=new ompl_ros_interface::OmplRos();
	//ompl_ros->run();    
}


void MainWindow::newHand(){
	bool ok;
	QString packageHand=QInputDialog::getText(this, "Select package ROS", "Package ROS", QLineEdit::Normal, "",&ok);
	if(ok && !packageHand.isEmpty()){
		std::string packagePath=ros::package::getPath(packageHand.toUtf8().constData());
		if(packagePath!="")
		{
			QFileDialog *dialog=new QFileDialog(this, "Path to new hand", packagePath.c_str(), "OSG Files (*.osg)");
			if(dialog->exec()){
				QString pathHand=dialog->selectedFiles()[0];
				QString nameHand=QInputDialog::getText(this, "Insert the name of the hand", "Hand's name", QLineEdit::Normal, "",&ok);
				if(ok && !nameHand.isEmpty()){
					int max=0;
					for(size_t i=0; i<handsDB.size(); i++){
						if(max<handsDB[i]->hand_id.data())
							max=handsDB[i]->hand_id.data();
					}
					Hands newHand;
					newHand.hand_id.data()=max+1;
					newHand.name_hand.data()=nameHand.toUtf8().constData();
					newHand.package_ros.data()=packageHand.toUtf8().constData();
					newHand.path.data()=pathHand.split(packagePath.c_str())[1].toUtf8().constData();
					if(!database->insertIntoDatabase(&newHand))
						std::cerr <<"Hand insertion failed"<<std::endl;
					else{
						
						ui.hands->addItem(newHand.name_hand.data().c_str());
						if(!database->getList(handsDB)){
							std::cerr<<"Failed to get list of hands"<<endl;
						}
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
			if(!database->getList(handsDB)){
				std::cerr<<"Failed to get list of hands"<<endl;
			}
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
	if(marker_cli!=NULL){
		delete(marker_cli);
		marker_cli=NULL;
	}
	if(ui.hands->currentRow()!=-1){
		int cont=0;

		while((handsDB[cont]->name_hand.data()!=ui.hands->item(ui.hands->currentRow())->text().toUtf8().constData())   &&  (cont < ui.hands->count())){
			cont++;
		}
		if(cont< ui.hands->count()){
			marker_cli= new InteractiveMarkerDisplay("osg_im","/basic_controls/update", scene->localizedWorld, *(frame_manager->getTFClient()));
			marker=new HandInteractiveMarker(handsDB[cont]->package_ros.data(),handsDB[cont]->path.data());
		}
	}
}


void MainWindow::deleteHand(){
	int cont=0;
	if(ui.hands->currentRow()!=-1){
		while((handsDB[cont]->name_hand.data()!=ui.hands->item(ui.hands->currentRow())->text().toUtf8().constData())   &&  (cont < ui.hands->count())){
			cont++;
		}
		if(cont< ui.hands->count()){
			if(!database->deleteFromDatabase(handsDB[cont].get())){
				std::cerr <<"Hand delete failed"<<std::endl;
			}
			int file=ui.hands->currentRow();
			ui.hands->setCurrentRow(-1);
			ui.hands->removeItemWidget(ui.hands->takeItem(file));
		}	
	}
}
void MainWindow::newPath(){
	if(grasp!=NULL){
		grasp->newPath(offsetp,offsetr,sceneBuilder);
	}
}


void MainWindow::rosSpin(){
	ros::spinOnce();
	double currSimTime = view->getFrameStamp()->getSimulationTime();
	double elapsed( currSimTime - prevSimTime );
	if(marker_cli!=NULL){
		marker_cli->update(elapsed, elapsed);
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
