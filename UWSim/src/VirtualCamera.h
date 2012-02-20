/*
 * VirtualCamera.h
 *
 *  Created on: 26/05/2010
 *      Author: mprats
 */

#ifndef VIRTUALCAMERA_H_
#define VIRTUALCAMERA_H_

#include "SimulatorConfig.h"
#include "CustomWidget.h"
#include "ConfigXMLParser.h"

#include <osgViewer/Viewer>
#include <osgGA/NodeTrackerManipulator>
#include <osg/Camera>
#include <osg/Texture2D>
#include <osgGA/GUIEventHandler>
#include <osg/Geometry>
#include <osg/NodeTrackerCallback>



/** A camera associated to a viewer */
class VirtualCamera : public CustomWidget
{

	//Custom node tracking
        class MyNodeTrackerCallback : public osg::NodeTrackerCallback {
	  osg::Camera *cam;

          void operator() (osg::Node *node, osg::NodeVisitor *nv)  {
		  //std::cerr << "Node Tracker callback" << std::endl;
		  osg::Matrixd m;
		  //((osg::Transform*)node)->computeWorldToLocalMatrix(m,nv);
		  m = osg::computeWorldToLocal(nv->getNodePath() );
          	  traverse(node,nv);
		  cam->setViewMatrix(m);
          }
	  public:
		void setCamera(osg::Camera *c) {this->cam=c;}
        };

public:
	std::string name;
	osg::ref_ptr<osg::Camera> textureCamera;
	osg::Node *trackNode;
	int width, height;
	double fx,fy,cx,cy;	///< intrinsic parameters
	double far,near,k;
	int paramsOn;

	osg::Image* renderTexture;
	//osg::ref_ptr<osg::Geometry> screenQuad;
	//osg::ref_ptr<osg::Geode> quadGeode;

    VirtualCamera(std::string name, osg::Node *trackNode, int width, int height);
    VirtualCamera(std::string name, osg::Node *trackNode, int width, int height, Parameters *params);
    VirtualCamera();
    void init(std::string name, osg::Node *trackNode, int width, int height, Parameters *params);

    void createCamera();

    //Interface to be implemented by widgets. Build a widget window with the data to be displayed
    osgWidget::Window* getWidgetWindow();
};

#endif /* VIRTUALCAMERA_H_ */
