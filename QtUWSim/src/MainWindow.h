#ifndef MAINWINDOW_H
#define MAINWINDOW_H
#include <QtGui/QApplication>
#include <QObject>
#include <QTimer>
#include <osgViewer/View>
#include <QMainWindow>
#include <QMenuBar>
#include <QMenu>
#include <QAction>
#include <QButtonGroup>
#include <QPushButton>
#include "QtViewerWidget.h"
#include "SceneBuilder.h"
#include "ViewBuilder.h"
#include <QDockWidget>
#include <QListWidget>
#include <QGridLayout>
#include <QListWidgetItem>
#include <QVector>
#include "PlanarGraspSpec.h"
#include <osg_utils/frame_manager.h>
#include <osg_interactive_markers/interactive_marker.h>
#include <osg_interactive_markers/interactive_marker_display.h>
#include "HandInteractiveMarker.h"
#include <database_interface/postgresql_database.h>
#include "handsDB.h"
#include "ompl_ros_interface/ompl_ros.h"
#include "GraspSpecification.h"

#include "ui_MainWindow.h"



using namespace osg_utils;
using namespace osg_interactive_markers;

class MainWindow : public QMainWindow
{
  
  Q_OBJECT
    
private:
  QImage *image;
  
  std::vector<boost::shared_ptr<Hands> > handsDB;

  
  
  
  osgViewer::View *view;
  osg::Group* root;
  osg::ref_ptr<osgOceanScene> scene;
  osgGA::CameraManipulator* oldManipulator; 
 
  QTimer timer;
  SceneBuilder *sceneBuilder;
  ViewBuilder *viewBuilder;
  ViewerWidget* viewWidget;
  PlanarGraspSpec* spec;
  HandInteractiveMarker *marker;
  database_interface::PostgresqlDatabase *database;
  GraspSpecification *grasp;
  
  boost::shared_ptr<FrameManager> frame_manager;
  InteractiveMarkerDisplay *marker_cli;
  double prevSimTime;
  std::vector<double> offsetr, offsetp;
  
  Ui::MainWindowClass ui;
  
  QLabel *openedFileName;
  

public:
 
  MainWindow(boost::shared_ptr<osg::ArgumentParser>);
  ~MainWindow();

  
public Q_SLOTS:
  void loadXML();
  void rosSpin();
  void createMosaic();
  void loadMosaic();
  void graspSpecification();
  void newHand();
  void handChanged();
  void deleteHand();
  void about();
  void createStatusBar();
  void updateStatusBar(const QString name);
  void newPath();
};

#endif





