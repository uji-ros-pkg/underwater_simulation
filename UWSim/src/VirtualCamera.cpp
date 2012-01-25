/*
 * VirtualCamera.cpp
 *
 *  Created on: 26/05/2010
 *      Author: mprats
 */

#include "VirtualCamera.h"
#include <iostream>

VirtualCamera::VirtualCamera(){}

void VirtualCamera::init(std::string name, osg::Node *trackNode, int width, int height, Parameters * params) {
	this->name=name;
	this->trackNode=trackNode;
	this->width=width;
	this->height=height;
	if(params!=NULL){
	  this->fx=params->fx;
	  this->fy=params->fy;
	  this->far=params->f;
	  this->near=params->n;
	  this->cx=params->x0;
	  this->cy=params->y0;
	  this->k=params->k;
	  this->paramsOn=1;
        }
	else
	  this->paramsOn=0;
        
	renderTexture=new osg::Image();
	renderTexture->allocateImage(width, height, 1, GL_RGB, GL_UNSIGNED_BYTE);

	createCamera();
}

VirtualCamera::VirtualCamera(std::string name, osg::Node *trackNode, int width, int height) {
	init(name, trackNode,width,height,NULL);
}

void VirtualCamera::createCamera()
{
	textureCamera = new osg::Camera;
	textureCamera->setReferenceFrame(osg::Transform::ABSOLUTE_RF);
	textureCamera->setClearMask(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	textureCamera->setViewport(0, 0, width, height);

	// Frame buffer objects are the best option
	textureCamera->setRenderTargetImplementation(osg::Camera::FRAME_BUFFER_OBJECT);

	// We need to render to the texture BEFORE we render to the screen
	textureCamera->setRenderOrder(osg::Camera::PRE_RENDER);

	// The camera will render into the texture that we created earlier
	textureCamera->attach(osg::Camera::COLOR_BUFFER, renderTexture);

	textureCamera->setName("CamViewCamera");
	textureCamera->setComputeNearFarMode(osg::CullSettings::DO_NOT_COMPUTE_NEAR_FAR);
	if(!paramsOn){
	  //set default fov, near and far parameters
	  textureCamera->setProjectionMatrixAsPerspective(50, 1.33, 0.18, 10);
	  osg::Matrixd m;
	  m=textureCamera->getProjectionMatrix();
	  fx=m(0,0)*width/2.0;
	  fy=m(1,1)*height/2.0;
	  cx=-(m(0,2)-1)*width/2.0;
	  cy=-(m(1,2)-1)*height/2.0;
	}
	else{	
	  //set opengl projection matrix from calibration parameters fx, fy, w, h, x0, y0, n
	  // How to obtain opengl projection matrix from camera calibration parameters:
	  // 2.0*fx/w    2.0*k/w    1-2*x0/w       0
	  // 0          2.0*fy/h   1-2*y0/h       0
	  // 0           0          (f+n)/(n-f)    2*fn/(n-f)
	  // 0           0         -1              0
	  //osg::Matrixd m(2.0*fx/width,2.0*k/width,1-(2*cx/width),0,0,2.0*fy/height,1-(2.0*cy/height),0,0,0,(far+near)/(near-far),2*far*near/(near-far),0,0,-1,0); //osg Uses trasposed matrix
	  osg::Matrixd m(2.0*fx/width,0,0,0,2.0*k/width,2.0*fy/height,0,0,1-(2*cx/width),1-(2.0*cy/height),(far+near)/(near-far),-1,0,0,2*far*near/(near-far),0);
	  textureCamera->setProjectionMatrix(m);

	}

	//make this camera track the node
	osg::ref_ptr<MyNodeTrackerCallback> node_tracker = new MyNodeTrackerCallback;
	node_tracker->setCamera(textureCamera);
	trackNode->setUpdateCallback(node_tracker);
}

osgWidget::Window* VirtualCamera::getWidgetWindow() {
	osgWidget::Box *box=new osgWidget::Box("VirtualCameraBox", osgWidget::Box::HORIZONTAL, true);
	osgWidget::Widget* widget = new osgWidget::Widget("VirtualCameraWidget", width, height);
	widget->setImage(renderTexture,true,false);
	box->addWidget(widget);
	osgWidget::Window* boxwin=box;
	boxwin->getBackground()->setColor(1.0f, 0.0f, 0.0f, 0.8f);
	boxwin->attachMoveCallback();
	boxwin->attachScaleCallback();
	return boxwin;
}
