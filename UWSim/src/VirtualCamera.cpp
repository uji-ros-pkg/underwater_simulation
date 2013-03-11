/*
 * VirtualCamera.cpp
 *
 *  Created on: 26/05/2010
 *      Author: mprats
 */

#include "VirtualCamera.h"
#include "UWSimUtils.h"
#include <iostream>
#include "SceneBuilder.h"

VirtualCamera::VirtualCamera(){}

void VirtualCamera::init(osg::Group *uwsim_root, std::string name, osg::Node *trackNode, int width, int height, double baseline, std::string frameId, Parameters *params,int range,double fov) {
	this->uwsim_root=uwsim_root;
	this->name=name;

	this->trackNode=trackNode;
	//Add a switchable frame geometry on the camera frame
        osg::ref_ptr<osg::Node> axis=UWSimGeometry::createSwitchableFrame();
	this->trackNode->asGroup()->addChild(axis);	

	this->width=width;
	this->height=height;
	this->baseline = baseline;
	this->frameId = frameId;
	this->fov=fov;
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
	this->range=range;
        
	renderTexture=new osg::Image();
	renderTexture->allocateImage(width, height, 1, GL_RGB, GL_UNSIGNED_BYTE);
	depthTexture=new osg::Image();
	depthTexture->allocateImage(width, height, 1, GL_DEPTH_COMPONENT, GL_FLOAT);

	createCamera();
}


VirtualCamera::VirtualCamera(osg::Group *uwsim_root, std::string name, osg::Node *trackNode, int width,double fov, double range){//Used in multibeam
  this->far=range*1.2; //Z-buffer has very low resolution near far plane so we extend it and cut far plane later.
  init(uwsim_root, name, trackNode,width,1,0.0, "", NULL,1,fov);

}

VirtualCamera::VirtualCamera(osg::Group *uwsim_root, std::string name, osg::Node *trackNode, int width, int height, double baseline, std::string frameId) {
	init(uwsim_root, name, trackNode,width,height,baseline, frameId, NULL,0,0);
}

VirtualCamera::VirtualCamera(osg::Group *uwsim_root, std::string name, osg::Node *trackNode, int width, int height, double baseline, std::string frameId, Parameters *params,int range) {
	init(uwsim_root, name, trackNode,width,height,baseline,frameId,params,range,0);
}

VirtualCamera::VirtualCamera(osg::Group *uwsim_root, std::string name, osg::Node *trackNode, int width, int height, Parameters *params) {
	init(uwsim_root, name, trackNode,width,height,0.0,"",params,0,0);
}

VirtualCamera::VirtualCamera(osg::Group *uwsim_root, std::string name, osg::Node *trackNode, int width, int height) {
	init(uwsim_root, name, trackNode,width,height,0.0,"", NULL,0,0);
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
	textureCamera->attach(osg::Camera::COLOR_BUFFER, renderTexture.get());
	textureCamera->attach(osg::Camera::DEPTH_BUFFER, depthTexture.get());

	textureCamera->setName("CamViewCamera");
	textureCamera->setComputeNearFarMode(osg::CullSettings::DO_NOT_COMPUTE_NEAR_FAR);
	
	if(!paramsOn){
	  //set default fov, near and far parameters
	  if(!fov)
	    textureCamera->setProjectionMatrixAsPerspective(50, 1.33, 0.18, 20);
          else //Used in multibeam, aspect ratio correction should be improved
	    textureCamera->setProjectionMatrixAsPerspective(fov, 1+0.004464*fov, 0.8, far);
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

	Tx = (-fx * baseline);
	Ty = 0.0;	

	node_tracker = new MyNodeTrackerCallback(uwsim_root, depthTexture, textureCamera);
	trackNode->setUpdateCallback(node_tracker);
}

osg::ref_ptr<osgWidget::Window> VirtualCamera::getWidgetWindow() {
	osg::ref_ptr<osgWidget::Box> box=new osgWidget::Box("VirtualCameraBox", osgWidget::Box::HORIZONTAL, true);
	osg::ref_ptr<osgWidget::Widget> widget = new osgWidget::Widget("VirtualCameraWidget", width, height);
	if(!range)
	  widget->setImage(renderTexture.get(),true,false);
	else
	  widget->setImage(depthTexture.get(),true,false);
	box->addWidget(widget.get());
	box->getBackground()->setColor(1.0f, 0.0f, 0.0f, 0.8f);
	box->attachMoveCallback();
	box->attachScaleCallback();
	return box;
}
