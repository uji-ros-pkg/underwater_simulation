#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QtGui/QApplication>
#include <QObject>
#include <QTimer>
#include <QMainWindow>
#include <QMenuBar>
#include <QMenu>
#include <QAction>
#include <QButtonGroup>
#include <QPushButton>
#include <QDockWidget>
#include <QListWidget>
#include <QGridLayout>
#include <QListWidgetItem>
#include <QVector>

#include <osgViewer/View>
#include <osg_utils/frame_manager.h>
#include <osg_interactive_markers/interactive_marker.h>
#include <osg_interactive_markers/interactive_marker_display.h>

#include "QtViewerWidget.h"
#include "SceneBuilder.h"
#include "ViewBuilder.h"
#include "PlanarGraspSpec.h"
#include "HandInteractiveMarker.h"
#include "GraspSpecification.h"
#include "ui_MainWindow.h"
//#include <database_interface/postgresql_database.h>
//#include "handsDB.h"
//#include "ompl_ros_interface/ompl_ros.h"

#include <sqlite3.h>

using namespace osg_utils;
using namespace osg_interactive_markers;

class MainWindow : public QMainWindow
{

	Q_OBJECT

private:
	QImage *image;

	//  std::vector<boost::shared_ptr<Hands> > handsDB;
	std::map<std::string, std::pair<std::string,std::string> > map;

	osg::Group* root;
	QTimer timer;
	//  database_interface::PostgresqlDatabase *database;
	sqlite3 *db; 					//FIXME: Use smart pointers
	ros::Time prevSimTime;
	std::vector<double> offsetr, offsetp;

	//Mosaic mode
	osg::ref_ptr<osgViewer::Viewer> mosaic_viewer_;	//Viewer when in mosaic mode
	std::vector<boost::shared_ptr<PlanarGraspSpec> > planar_grasp_spec_;
	boost::shared_ptr<GraspSpecification> grasp;
	std::map<QListWidgetItem*, int > widget_to_spec_;
	osgGA::CameraManipulator* oldManipulator;
	osg::Switch* switches;


	//Scene mode
	boost::shared_ptr<SceneBuilder> sceneBuilder;
	osg::ref_ptr<osgOceanScene> scene;
	boost::shared_ptr<ViewBuilder> viewBuilder;
	boost::shared_ptr<ViewerWidget> viewWidget;

	//Interactive markers
	boost::shared_ptr<HandInteractiveMarker> marker;
	boost::shared_ptr<FrameManager> frame_manager;
	boost::shared_ptr<InteractiveMarkerDisplay> joint_marker_cli;
	boost::shared_ptr<InteractiveMarkerDisplay> hand_marker_cli;

	Ui::MainWindowClass ui;

	QLabel *openedFileName;

	boost::shared_ptr<osg::ArgumentParser> arguments_;

	void updateDBHandsList();
	bool deleteDBHand(std::string hand_name);
	bool insertDBHand(std::string hand_name, std::string hand_package, std::string hand_path);


public:

	MainWindow(boost::shared_ptr<osg::ArgumentParser>);
	~MainWindow();


public Q_SLOTS:
void loadXML();
void rosSpin();
void createMosaic();
void loadMosaic();

void showIntervention2DPanel();
void newIntervention2D();
void deleteIntervention2D();
void graspIntervention2D();
void selectedIntervention2D();
void exportIntervention2D();

void graspSpecification();
void newHand();
void handChanged();
void deleteHand();
void about();
void createStatusBar();
void updateStatusBar(const QString name);
void sceneConfiguration();
};

#endif





