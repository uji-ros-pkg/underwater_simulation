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
	UpdateLMVPM(osg::Node* node,double fov)
		: mNode(node), mfov(fov)
	{
	}
	virtual void operator () (osg::Uniform* u, osg::NodeVisitor*)
	{
    osg::Vec3  position=getWorldCoords(mNode)->getTrans();
    osg::Quat rotation=getWorldCoords(mNode)->getRotate();
    osg::Vec3 up(0.0f,-1.0f,0.0f);
    osg::Vec3 direction(0.0f, 0.0f, -1.0f);
    osg::Matrixd lmvpm =osg::Matrixd::lookAt(position, position+rotation*direction, rotation*up) * osg::Matrixd::perspective(mfov,1.0,0.1,100)*osg::Matrixd::translate(1.0,1.0,1.0);

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
    init(name, root, node, image_name, range, fov, visible);
}

VirtualSLSProjector::VirtualSLSProjector(std::string name, osg::Node *root, osg::Node *node, std::string image_name, double fov, bool visible) {
	double range = 0; 
    init(name, root, node, image_name, range, fov, visible);
}

void VirtualSLSProjector::init(std::string name, osg::Node *root, osg::Node *node, std::string image_name, double range, double fov, bool visible) {
	this->name = name;
    this->fov = fov;
    this->range = range;
    this->node = node;
    this->image_name = image_name;
    this->visible = visible;
    this->lightNum = 1;
    this->textureUnit = 9;//1;
    osg::Vec3 position(0.0f,0.0f,0.0f);
    //osg::Vec3 direction(1.0f, 0.0f, 0.0f);
    osg::Vec3 direction(0.0f, 0.0f, -1.0f);//NEW CONVENTION

    // Add SLS projector!
    this->node->asGroup()->addChild(createSLNode(position, direction, fov, lightNum, textureUnit));

    //Add a switchable frame geometry on the sensor frame
    //osg::ref_ptr<osg::Node> axis=UWSimGeometry::createSwitchableFrame();
    //this->node->asGroup()->addChild(axis);

    project_on(root);	
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
    light2->setDirection(osg::Vec3(0,1,0));
    light2->setConstantAttenuation(0.0f);
    light2->setLinearAttenuation(0.0f);
    light2->setQuadraticAttenuation(0.0f);
    group->addChild(lightsource2);

    
    // create tex gen. 
    //osg::Vec3 up(0.0f,0.0f,1.0f);
    osg::Vec3 up(0.0f,-1.0f,0.0f); //NEW CONVENTION
    up = (direction ^ up) ^ direction;
    up.normalize();
    
    osg::TexGenNode* texgenNode = new osg::TexGenNode;
    texgenNode->setTextureUnit(textureUnit);
    osg::TexGen* texgen = texgenNode->getTexGen();
    texgen->setMode(osg::TexGen::EYE_LINEAR);       
    //                                                 EYE          CENTER         UP
    texgen->setPlanesFromMatrix(osg::Matrixd::lookAt(position, position+direction, up)*
                                osg::Matrixd::perspective(angle,1.0,0.1,100)*
                                osg::Matrixd::translate(1.0,1.0,1.0)*
                                //osg::Matrixd::translate(0.0,0.0,0.0)*
                                osg::Matrixd::scale(0.5,0.5,0.5));
                                //osg::Matrixd::scale(1,1,1));

    group->addChild(texgenNode);
    return group;
}

osg::StateSet* VirtualSLSProjector::createSLDecoratorState(osg::StateSet* stateset, unsigned int lightNum, unsigned int textureUnit)
{
    stateset->setMode(GL_LIGHT0+lightNum, osg::StateAttribute::ON);

    osg::Vec4 ambientColour(0.1f,0.1f,0.1f,0.1f);
    //osg::Vec4 ambientColour(0.05f,0.05f,0.05f,1.0f); 

    // set up spot light texture
    osg::Texture2D* texture = new osg::Texture2D();
    /*osg::Image* texture_to_project = osgDB::readImageFile(this->image_name);
    assert(texture_to_project);*/
    texture->setTextureSize(512, 512);
    //texture->setImage(texture_to_project);
    texture->setBorderColor(osg::Vec4(ambientColour));
    texture->setWrap(osg::Texture::WRAP_S,osg::Texture::CLAMP_TO_BORDER); // fa que la textura no es repeteixi continuament
    texture->setWrap(osg::Texture::WRAP_T,osg::Texture::CLAMP_TO_BORDER); // veure: http://lucera-project.blogspot.com.es/2010/06/opengl-wrap.html
    texture->setWrap(osg::Texture::WRAP_R,osg::Texture::CLAMP_TO_BORDER);
		texture->setInternalFormat(GL_DEPTH_COMPONENT);
		texture->setShadowTextureMode(osg::Texture2D::LUMINANCE);
		texture->setShadowComparison(true);
		texture->setShadowCompareFunc(osg::Texture::LEQUAL);
		texture->setFilter(osg::Texture::MIN_FILTER,osg::Texture::LINEAR);
		texture->setFilter(osg::Texture::MAG_FILTER,osg::Texture::LINEAR);

    //Camera
    osg::Camera* camera = new osg::Camera;
    //camera->setReferenceFrame(osg::Camera::ABSOLUTE_RF);
    camera->setProjectionMatrixAsPerspective(70, 1, 0.001, 100 );
    camera->setViewport(0,0,512,512);
    camera->setRenderTargetImplementation(osg::Camera::FRAME_BUFFER_OBJECT);
    camera->setClearColor(osg::Vec4(1.f,1.f,1.f,1.0f));
    camera->setClearMask(GL_DEPTH_BUFFER_BIT);
    camera->setComputeNearFarMode(osg::CullSettings::DO_NOT_COMPUTE_NEAR_FAR);
    //camera->addChild(mx2);
    camera->setRenderOrder(osg::Camera::PRE_RENDER);
    camera->attach(osg::Camera::DEPTH_BUFFER,texture);
    camera->setCullingActive(true);
    node->asGroup()->addChild(camera);
    //camera->setUpdateCallback( new AttachNodeUpdateCallback(mx) );

    osg::Vec3  position=getWorldCoords(node)->getTrans();
    osg::Vec3 up(0.0f,-1.0f,0.0f);
    osg::Vec3 direction(0.0f, 0.0f, -1.0f);
    osg::Matrixd lmvpm =osg::Matrixd::lookAt(position, position+direction, up) * osg::Matrixd::perspective(fov,1.0,0.1,100)*
                                osg::Matrixd::translate(1.0,1.0,1.0)*
                                osg::Matrixd::scale(0.5,0.5,0.5);
    osg::Uniform* u = new osg::Uniform("LightModelViewProjectionMatrix",lmvpm);
    u->setUpdateCallback( new UpdateLMVPM(node,fov) );
    stateset->addUniform( u );
    
    stateset->setTextureAttributeAndModes(textureUnit, texture, osg::StateAttribute::ON );
    
    // set up tex gens
    stateset->setTextureMode(textureUnit, GL_TEXTURE_GEN_S, osg::StateAttribute::ON); //Generació automàtica de 
    stateset->setTextureMode(textureUnit, GL_TEXTURE_GEN_T, osg::StateAttribute::ON); //les coordenades de les textures
    stateset->setTextureMode(textureUnit, GL_TEXTURE_GEN_R, osg::StateAttribute::ON); //per mapejar correctament aquestes
    stateset->setTextureMode(textureUnit, GL_TEXTURE_GEN_Q, osg::StateAttribute::ON); //als models en 3D
    
    //TODO: transparency
    //TODO: light
    //TODO: project only on the scene, not in vehicle
    //Notes: a sobre d'un color hi ha textura, a sobre de negre, no.

    osg::BlendFunc *blendFunc = new osg::BlendFunc;
    //blendFunc->setFunction(GL_SRC_ALPHA, GL_ONE); //oceà blanc, linies a la part blanca de la caixa
    blendFunc->setFunction(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA); // oceà blau, igual que abans
    //blendFunc->setFunction(GL_SRC_COLOR,GL_ONE_MINUS_CONSTANT_COLOR); // oceà blau clar, igual
    //blendFunc->setFunction(GL_SRC_COLOR,GL_CONSTANT_COLOR); // ocea negre/blau, igual.
    //stateset->setMode( GL_LIGHTING, osg::StateAttribute::OFF );

    //stateset->setMode( GL_DEPTH_TEST, osg::StateAttribute::OFF ); // actiu per defecte, si el descativo no es dibuxa res a la caixa
    //stateset->setMode( GL_CULL, osg::StateAttribute::OFF );

    //stateset->setMode( GL_BLEND, osg::StateAttribute::ON );
    stateset->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
    //stateset->setAttributeAndModes( blendFunc, osg::StateAttribute::ON );
    


    return stateset;
}

void VirtualSLSProjector::project_on(osg::Node* canvas)
{
    osg::StateSet* stateset = canvas->getOrCreateStateSet();
    canvas->setStateSet(createSLDecoratorState(stateset,lightNum,textureUnit));
    //std::cout << stateset->getActiveTextureUnit() << std::endl;
}
