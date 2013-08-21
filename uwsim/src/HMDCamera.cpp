/*
 * HMDCamera.cpp
 *
 *  Created on: Jun 30, 2013
 *      Author: janoc
 */

#include <osg/io_utils>
#include <osg/Texture2D>
#include <osg/PolygonMode>
#include <osg/Program>
#include <osg/Shader>

#include <osgDB/ReadFile>
#include <osgViewer/View>

#include "HMDCamera.h"
#include "oculusdevice.h"


HMDCamera::HMDCamera(osgViewer::View *view, OculusDevice *dev) : osg::Group(),
                                                                 m_configured(false),
                                                                 m_view(view),
                                                                 m_dev(dev)

{}


HMDCamera::~HMDCamera()
{
}

void HMDCamera::traverse(osg::NodeVisitor& nv)
{
    if(!m_configured)
    {
        m_configured = true;
        configure();
    }

    // Get orientation from oculus sensor
    osg::Matrixf orient = osg::Matrix::rotate(m_dev->getOrientation());

    // Nasty hack to update the view offset for each of the slave cameras
    // There doesn't seem to be an accessor for this, fortunately the offsets are public
    m_view->findSlaveForCamera(m_l_rtt.get())->_viewOffset = orient;
    m_view->findSlaveForCamera(m_r_rtt.get())->_viewOffset = orient;

    osg::Group::traverse(nv);
}

osg::Camera* HMDCamera::createRTTCam(osg::Camera::BufferComponent buffer, osg::Texture* tex)
{
    osg::ref_ptr<osg::Camera> camera = new osg::Camera;

    camera->setClearColor(osg::Vec4(0.2f, 0.2f, 0.4f, 1.0f));
    camera->setClearMask( GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT );
    camera->setRenderTargetImplementation( osg::Camera::FRAME_BUFFER_OBJECT );
    camera->setRenderOrder( osg::Camera::PRE_RENDER );
    camera->setGraphicsContext(m_view->getCamera()->getGraphicsContext());

    camera->setComputeNearFarMode( osg::CullSettings::DO_NOT_COMPUTE_NEAR_FAR );
    camera->setReferenceFrame(osg::Camera::RELATIVE_RF);

    if ( tex )
    {
        tex->setFilter( osg::Texture2D::MIN_FILTER, osg::Texture2D::LINEAR );
        tex->setFilter( osg::Texture2D::MAG_FILTER, osg::Texture2D::LINEAR );
        camera->setViewport( 0, 0, tex->getTextureWidth(), tex->getTextureHeight() );
        camera->attach( buffer, tex, 0, 0, false, 4, 4);
    }

    return camera.release();
}

osg::Camera* HMDCamera::createHUDCam(double left, double right, double bottom, double top)
{
    osg::ref_ptr<osg::Camera> camera = new osg::Camera;
    camera->setGraphicsContext(m_view->getCamera()->getGraphicsContext());
    camera->setReferenceFrame( osg::Transform::ABSOLUTE_RF );
    camera->setClearColor(osg::Vec4(0.2f, 0.2f, 0.4f, 1.0f));
    camera->setClearMask( GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);
    camera->setRenderOrder( osg::Camera::POST_RENDER );
    camera->setAllowEventFocus( false );
    camera->setProjectionMatrix( osg::Matrix::ortho2D(left, right, bottom, top) );
    camera->getOrCreateStateSet()->setMode( GL_LIGHTING, osg::StateAttribute::OFF );

    return camera.release();
}

osg::Geode *HMDCamera::createHUDQuad( float width, float height, float scale )
{
    osg::Geometry* geom = osg::createTexturedQuadGeometry(osg::Vec3(),
                                                          osg::Vec3(width, 0.0f, 0.0f),
                                                          osg::Vec3(0.0f,  height, 0.0f),
                                                          0.0f, 0.0f, width*scale, height*scale );
    osg::ref_ptr<osg::Geode> quad = new osg::Geode;
    quad->addDrawable( geom );

    int values = osg::StateAttribute::OFF | osg::StateAttribute::PROTECTED;
    quad->getOrCreateStateSet()->setAttribute(new osg::PolygonMode(osg::PolygonMode::FRONT_AND_BACK, osg::PolygonMode::FILL), values );
    quad->getOrCreateStateSet()->setMode( GL_LIGHTING, values );
    return quad.release();
}

void HMDCamera::configure()
{
    const int textureWidth  = m_dev->scaleFactor() * m_dev->hScreenResolution()/2;
    const int textureHeight = m_dev->scaleFactor() * m_dev->vScreenResolution();

    // master projection matrix
    m_view->getCamera()->setProjectionMatrix(m_dev->projectionCenterMatrix());

    osg::ref_ptr<osg::Texture2D> l_tex = new osg::Texture2D;
    l_tex->setTextureSize( textureWidth, textureHeight );
    l_tex->setInternalFormat( GL_RGBA );

    osg::ref_ptr<osg::Texture2D> r_tex = new osg::Texture2D;
    r_tex->setTextureSize( textureWidth, textureHeight );
    r_tex->setInternalFormat( GL_RGBA );

    osg::ref_ptr<osg::Camera> l_rtt = createRTTCam(osg::Camera::COLOR_BUFFER, l_tex);
    l_rtt->setComputeNearFarMode(osg::CullSettings::DO_NOT_COMPUTE_NEAR_FAR);
    l_rtt->setReferenceFrame(osg::Camera::RELATIVE_RF);
    m_l_rtt = l_rtt;

    osg::ref_ptr<osg::Camera> r_rtt = createRTTCam(osg::Camera::COLOR_BUFFER, r_tex);
    r_rtt->setComputeNearFarMode( osg::CullSettings::DO_NOT_COMPUTE_NEAR_FAR );
    r_rtt->setReferenceFrame(osg::Camera::RELATIVE_RF);
    m_r_rtt = r_rtt;

    // Create HUD cameras for each eye
    osg::ref_ptr<osg::Camera> l_hud = createHUDCam(0.0, 1.0, 0.0, 1.0);
    l_hud->setViewport(new osg::Viewport(0, 0, m_dev->hScreenResolution() / 2.0f, m_dev->vScreenResolution()));

    osg::ref_ptr<osg::Camera> r_hud = createHUDCam(0.0, 1.0, 0.0, 1.0);
    r_hud->setViewport(new osg::Viewport(m_dev->hScreenResolution() / 2.0f, 0,
                                         m_dev->hScreenResolution() / 2.0f, m_dev->vScreenResolution()));

    // Create quads on each camera
    osg::ref_ptr<osg::Geode> leftQuad = createHUDQuad(1.0f, 1.0f);
    l_hud->addChild(leftQuad);

    osg::ref_ptr<osg::Geode> rightQuad = createHUDQuad(1.0f, 1.0f);
    r_hud->addChild(rightQuad);

    // Set up shaders from the Oculus SDK documentation
    osg::ref_ptr<osg::Program> program = new osg::Program;
    osg::ref_ptr<osg::Shader> vertexShader = new osg::Shader(osg::Shader::VERTEX);
    vertexShader->loadShaderSourceFromFile("warp.vert");

    osg::ref_ptr<osg::Shader> fragmentShader = new osg::Shader(osg::Shader::FRAGMENT);
    //fragmentShader->loadShaderSourceFromFile("warp.frag");
    fragmentShader->loadShaderSourceFromFile("warpWithChromeAb.frag");

    program->addShader(vertexShader);
    program->addShader(fragmentShader);

    // Configure state sets for both eyes
    osg::StateSet* leftEyeStateSet = leftQuad->getOrCreateStateSet();
    leftEyeStateSet->setTextureAttributeAndModes(0, l_tex, osg::StateAttribute::ON);
    leftEyeStateSet->setAttributeAndModes( program, osg::StateAttribute::ON );
    leftEyeStateSet->addUniform( new osg::Uniform("WarpTexture", 0) );
    leftEyeStateSet->addUniform( new osg::Uniform("LensCenter", m_dev->lensCenter(OculusDevice::LEFT_EYE)));
    leftEyeStateSet->addUniform( new osg::Uniform("ScreenCenter", m_dev->screenCenter()));
    leftEyeStateSet->addUniform( new osg::Uniform("Scale", m_dev->scale()));
    leftEyeStateSet->addUniform( new osg::Uniform("ScaleIn", m_dev->scaleIn()));
    leftEyeStateSet->addUniform( new osg::Uniform("HmdWarpParam", m_dev->warpParameters()));
    leftEyeStateSet->addUniform( new osg::Uniform("ChromAbParam", m_dev->chromAbParameters()));

    osg::StateSet* rightEyeStateSet = rightQuad->getOrCreateStateSet();
    rightEyeStateSet->setTextureAttributeAndModes(0, r_tex, osg::StateAttribute::ON);
    rightEyeStateSet->setAttributeAndModes( program, osg::StateAttribute::ON );
    rightEyeStateSet->addUniform( new osg::Uniform("WarpTexture", 0) );
    rightEyeStateSet->addUniform( new osg::Uniform("LensCenter", m_dev->lensCenter(OculusDevice::RIGHT_EYE)));
    rightEyeStateSet->addUniform( new osg::Uniform("ScreenCenter", m_dev->screenCenter()));
    rightEyeStateSet->addUniform( new osg::Uniform("Scale", m_dev->scale()));
    rightEyeStateSet->addUniform( new osg::Uniform("ScaleIn", m_dev->scaleIn()));
    rightEyeStateSet->addUniform( new osg::Uniform("HmdWarpParam", m_dev->warpParameters()));
    rightEyeStateSet->addUniform( new osg::Uniform("ChromAbParam", m_dev->chromAbParameters()));

    // Add cameras as slaves, specifying offsets for the projection
    // View takes ownership of our cameras, that's why we keep only weak pointers to them
    m_view->addSlave(l_rtt, m_dev->projectionOffsetMatrix(OculusDevice::LEFT_EYE), osg::Matrixf::identity(), true);
    m_view->addSlave(r_rtt, m_dev->projectionOffsetMatrix(OculusDevice::RIGHT_EYE), osg::Matrixf::identity(), true);

    m_view->addSlave(l_hud, false);
    m_view->addSlave(r_hud, false);
}
