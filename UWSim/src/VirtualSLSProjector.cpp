/*
 * VirtualStructuredLightProjector.cpp
 *
 *  Created on: 06/02/2013
 *      Author: Miquel Massot
 *
 */

#include "VirtualSLSProjector.h"
#include "UWSimUtils.h"
#include "osg/BlendFunc"
#include <iostream>
#include <assert.h>

class UpdateLMVPM : public osg::Uniform::Callback
{
public:
	UpdateLMVPM(osg::Camera* camera)
		: mCamera(camera)
	{
	}
	virtual void operator () (osg::Uniform* u, osg::NodeVisitor*)
	{
		osg::Matrixd lmvpm =
				mCamera->getViewMatrix() * mCamera->getProjectionMatrix() * 
                osg::Matrix::translate( 1,1,1 ) * osg::Matrix::scale( 0.5, 0.5, 0.5 );

		u->set(lmvpm);
	}

protected:
	osg::Camera* mCamera;
};

class UpdateTextureLMVPM : public osg::Uniform::Callback
{
public:
  UpdateTextureLMVPM(osg::Node* node,double fov)
		: mNode(node), mfov(fov)
  {
  }
  virtual void operator () (osg::Uniform* u, osg::NodeVisitor*)
  {
    osg::Vec3 position=getWorldCoords(mNode)->getTrans();
    osg::Quat rotation=getWorldCoords(mNode)->getRotate();
    osg::Vec3 up(0.0f,-1.0f,0.0f);
    osg::Vec3 direction(0.0f, 0.0f, -1.0f);
    osg::Matrixd lmvpm =osg::Matrixd::lookAt(position, position+rotation*direction, rotation*up) * osg::Matrixd::perspective(mfov,1.0,0.1,100)*osg::Matrixd::translate(0.5,0.5,0.5)* osg::Matrix::scale( 0.5, 0.5, 0.5 );

u->set(lmvpm);
  }
protected:
	osg::Node* mNode;
double mfov;
}; 


VirtualSLSProjector::VirtualSLSProjector(){
    osg::ref_ptr<osg::Node> node = new osg::Node;
    osg::ref_ptr<osg::Node> root = new osg::Node;
    std::string name = "SLSprojector";
    std::string image_name = "laser_texture.png";
    double range = 0; //TODO: Not implemented 
    double fov = 60.0;
    bool visible = 1; //TODO: Not implemented
    init(name, root, node, image_name, range, fov, visible,VirtualCamera());
}

VirtualSLSProjector::VirtualSLSProjector(std::string name, osg::Node *root, osg::Node *node, std::string image_name, double fov, bool visible,VirtualCamera camera) {
	double range = 0; 
    init(name, root, node, image_name, range, fov, visible,camera);
}

void VirtualSLSProjector::init(std::string name, osg::Node *root, osg::Node *node, std::string image_name, double range, double fov, bool visible,VirtualCamera camera) {
	this->name = name;
    this->fov = fov;
    this->range = range;
    this->node = node;
    this->image_name = image_name;
    this->visible = visible;
    this->lightNum = 1;
    this->textureUnit = 3;//1;
    osg::Vec3 position(0.0f,0.0f,0.0f);
    //osg::Vec3 direction(1.0f, 0.0f, 0.0f);
    osg::Vec3 direction(0.0f, 0.0f, -1.0f);//NEW CONVENTION

    // Add SLS projector!
    this->node->asGroup()->addChild(createSLNode(position, direction, fov, lightNum, textureUnit));

    //Create depth texture for shadow mapping test
    dbgDepthTexture = new osg::Texture2D;
    dbgDepthTexture->setTextureSize(512, 512);  //CHECK: Maybe we can use a smaller texture?
    dbgDepthTexture->setInternalFormat(GL_DEPTH_COMPONENT);
    dbgDepthTexture->setFilter(osg::Texture::MIN_FILTER,osg::Texture::LINEAR);
    dbgDepthTexture->setFilter(osg::Texture::MAG_FILTER,osg::Texture::LINEAR);
    root->getOrCreateStateSet()->setTextureAttributeAndModes(3,dbgDepthTexture,osg::StateAttribute::ON);
    camera.textureCamera->attach(osg::Camera::DEPTH_BUFFER,dbgDepthTexture);

    osg::Texture2D* texture = new osg::Texture2D();
    osg::Image* texture_to_project = osgDB::readImageFile(this->image_name);
    assert(texture_to_project);
    texture->setImage(texture_to_project);
    texture->setWrap(osg::Texture::WRAP_S,osg::Texture::CLAMP_TO_BORDER); // fa que la textura no es repeteixi continuament
    texture->setWrap(osg::Texture::WRAP_T,osg::Texture::CLAMP_TO_BORDER); // veure: http://lucera-project.blogspot.com.es/2010/06/opengl-wrap.html
    texture->setWrap(osg::Texture::WRAP_R,osg::Texture::CLAMP_TO_BORDER);
    root->getOrCreateStateSet()->setTextureAttributeAndModes(4,texture,osg::StateAttribute::ON);

    /*camera = new osg::Camera;
    camera->setReferenceFrame(osg::Camera::ABSOLUTE_RF);
    camera->setProjectionMatrixAsPerspective(70, 1, 0.001, 100 ); 
    camera->setViewport(0,0,512,512);
    camera->setRenderTargetImplementation(osg::Camera::FRAME_BUFFER_OBJECT);
    camera->setClearColor(osg::Vec4(1.f,1.f,1.f,1.0f));
    camera->setClearMask(GL_DEPTH_BUFFER_BIT);
    camera->setComputeNearFarMode(osg::CullSettings::DO_NOT_COMPUTE_NEAR_FAR);
    //camera->addChild(mx2);
    camera->setRenderOrder(osg::Camera::PRE_RENDER);
    camera->attach(osg::Camera::DEPTH_BUFFER,dbgDepthTexture);
    camera->setCullingActive(true);
    //camera->setUpdateCallback( new AttachNodeUpdateCallback(mx) );*/


		osg::Matrixd lmvpm =
				camera.textureCamera->getViewMatrix() * camera.textureCamera->getProjectionMatrix() * 
                osg::Matrix::translate( 1,1,1 ) * osg::Matrix::scale( 0.5, 0.5, 0.5 );
    //root->asGroup()->addChild(camera);
    osg::Uniform* u = new osg::Uniform("LightModelViewProjectionMatrix",lmvpm);
    u->setUpdateCallback( new UpdateLMVPM(camera.textureCamera) );
    osg::Uniform* u2 = new osg::Uniform("textureViewProjectionMatrix",lmvpm);
    u2->setUpdateCallback( new UpdateTextureLMVPM(node,fov) );
    root->getOrCreateStateSet()->addUniform( u );
    root->getOrCreateStateSet()->addUniform( u2 );


}

osg::Node* VirtualSLSProjector::createSLNode(const osg::Vec3& position, const osg::Vec3& direction, float angle, unsigned int lightNum, unsigned int textureUnit)
{
    osg::Group* group = new osg::Group;
    
    // create light source.
    osg::LightSource* lightsource = new osg::LightSource;
    osg::Light* light = lightsource->getLight();
    light->setLightNum(lightNum);
    light->setPosition(osg::Vec4(position,1.0f));
    light->setAmbient(osg::Vec4(0.00f,0.00f,0.05f,1.0f));
    light->setDiffuse(osg::Vec4(1.0f,1.0f,1.0f,1.0f));
    light->setDirection(osg::Vec3(0,0,-1));
    light->setConstantAttenuation(0.0f);
    light->setLinearAttenuation(0.0f);
    light->setQuadraticAttenuation(0.0f);
    group->addChild(lightsource);

    // create light source2.
    osg::LightSource* lightsource2 = new osg::LightSource;
    osg::Light* light2 = lightsource2->getLight();
    light2->setLightNum(lightNum+1);
    light2->setPosition(osg::Vec4(position,1.0f));
    light2->setAmbient(osg::Vec4(0.00f,0.00f,0.05f,1.0f));
    light2->setDiffuse(osg::Vec4(0.0f,0.0f,1.0f,1.0f));
    light2->setDirection(osg::Vec3(1,0,0));
    light2->setConstantAttenuation(0.0f);
    light2->setLinearAttenuation(0.0f);
    light2->setQuadraticAttenuation(0.0f);
    group->addChild(lightsource2);

    return group;
}

