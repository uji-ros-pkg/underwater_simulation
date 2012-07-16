#ifndef QT_VIEWER_WIDGET_H
#define QT_VIEWER_WIDGET_H
#include <QtGui>

#include <osgViewer/CompositeViewer>
#include <osgViewer/ViewerEventHandlers>

#include <osgGA/TrackballManipulator>


#include <osgDB/ReadFile>
#include <osgDB/WriteFile>

#include <osgQt/GraphicsWindowQt>


#include <iostream>
using namespace std;

class ViewerWidget : public QWidget, public osgViewer::CompositeViewer
{
 public:
  ViewerWidget(osgViewer::View *view, osgViewer::ViewerBase::ThreadingModel threadingModel=osgViewer::CompositeViewer::SingleThreaded) : QWidget()
    {
      setThreadingModel(threadingModel);
      
      widget = addViewWidget(view );	
      QGridLayout* grid = new QGridLayout;
      grid->addWidget( widget, 0, 0 );
      setLayout( grid );
      _view=view;
 
      
      
      
      
      connect( &_timer, SIGNAL(timeout()), this, SLOT(update()) );
      _timer.start( 10 );
    }
    
  QWidget* addViewWidget( osgViewer::View *view )
   {
	addView( view );
	osg::DisplaySettings* ds = osg::DisplaySettings::instance().get();
	osg::ref_ptr<osg::GraphicsContext::Traits> traits = new osg::GraphicsContext::Traits;
	traits->windowName = "";
	traits->windowDecoration = true;
	traits->x = 0;
	traits->y = 0;
	traits->width = 500;
	traits->height = 500;
	traits->doubleBuffer = true;
	traits->alpha = ds->getMinimumNumAlphaBits();
	traits->stencil = ds->getMinimumNumStencilBits();
	traits->sampleBuffers = ds->getMultiSamples();
	traits->samples = ds->getNumMultiSamples();
	view->getCamera()->setGraphicsContext( new osgQt::GraphicsWindowQt(traits.get()) );
	view->getCamera()->setClearColor( osg::Vec4(0.2, 0.2, 0.6, 1.0) );
	view->getCamera()->setViewport( new osg::Viewport(0, 0, traits->width, traits->height) );
	view->getCamera()->setProjectionMatrixAsPerspective(
							    80.0f, static_cast<double>(traits->width)/static_cast<double>(traits->height), 1.0f, 10000.0f );
	
	osgQt::GraphicsWindowQt* gw = dynamic_cast<osgQt::GraphicsWindowQt*>( view->getCamera()->getGraphicsContext());
	
	return gw ? gw->getGLWidget() : NULL;
   }
 void updateViewerWidget(osgViewer::View *view){
	  
	  widget = addViewWidget(view );
	  _view->getCamera()->setNodeMask(0x0);
	  _view=view;
      QGridLayout* grid = new QGridLayout;
      grid->addWidget( widget, 0, 0 );
      delete(layout());
      setLayout( grid );
      
    }

    virtual void paintEvent( QPaintEvent* event )
    { 
		frame();

			
	}
		 

    
 private:
	/*osg::Camera* createCamera(){
		osg::DisplaySettings* ds = osg::DisplaySettings::instance().get();
		osg::ref_ptr<osg::GraphicsContext::Traits> traits = new osg::GraphicsContext::Traits;
		traits->windowName = "";
		traits->windowDecoration = true;
		traits->x = 0;
		traits->y = 0;
		traits->width = 500;
		traits->height = 500;
		traits->doubleBuffer = true;
		traits->alpha = ds->getMinimumNumAlphaBits();
		traits->stencil = ds->getMinimumNumStencilBits();
		traits->sampleBuffers = ds->getMultiSamples();
		traits->samples = ds->getNumMultiSamples();
		osg::ref_ptr<osg::Camera> camera=new osg::Camera;
		camera->setGraphicsContext( new osgQt::GraphicsWindowQt(traits.get()) );
		camera->setClearColor(osg::Vec4(0.2,0.2,0.6,1.0));
		camera->setViewport(new osg::Viewport(0,0 ,traits->width, traits->height));
		camera->setProjectionMatrixAsPerspective(80.0f, static_cast<double>(traits->width)/static_cast<double>(traits->height), 1.0f, 10000.0f );
		return camera;
	}*/
    
    QTimer _timer;
    osgViewer::View *_view;
    QWidget *widget;

};


#endif
