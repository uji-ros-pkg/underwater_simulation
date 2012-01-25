#include "HUDCamera.h"
#include <string.h>

HUDCamera::HUDCamera(unsigned int width, unsigned int height, unsigned int posx, unsigned int posy, double scale) {
	osg_image=NULL;
	this->width=width;
	this->height=height;
	this->posx=posx;
	this->posy=posy;
	this->scale=scale;
	osg_image=new osg::Image();
	osg_image->allocateImage(width, height, 1, GL_RGB, GL_UNSIGNED_BYTE);
	memset(osg_image->data(),0,width*height*3*sizeof(unsigned char));

	ready_=false;
	//OSG_INFO << "HUDCamera::HUDCamera Constructor finished " << info_topic << std::endl;
}

osgWidget::Window* HUDCamera::getWidgetWindow() {
	osgWidget::Box *box=new osgWidget::Box("HUDCameraBox", osgWidget::Box::HORIZONTAL, true);
	widget = new osgWidget::Widget("HUDCameraWidget", width, height);
	widget->setUpdateCallback(new widgetUpdateCallback(osg_image));
	//widget->setImage(osg_image,true,false);
	box->addWidget(widget);
	osgWidget::Window* boxwin=box;
	boxwin->setX(posx);
	boxwin->setY(posy);
	boxwin->setScale(scale);
	boxwin->getBackground()->setColor(1.0f, 0.0f, 0.0f, 0.8f);
	boxwin->attachMoveCallback();
	boxwin->attachScaleCallback();
	return boxwin;
}

HUDCamera::~HUDCamera() {
	//if (osg_image!=NULL) delete osg_image;
}

