/*
 * HMDCamera.h
 *
 *  Created on: Jun 30, 2013
 *      Author: janoc
 */

#ifndef HMDCAMERA_H_
#define HMDCAMERA_H_

#include <osg/Group>

class OculusDevice;
namespace osgViewer
{
    class View;
    class NodeVisitor;
}

class HMDCamera: public osg::Group
{
    public:
        HMDCamera(osgViewer::View *view, OculusDevice *dev);
        virtual void traverse(osg::NodeVisitor& nv);

    protected:
        virtual ~HMDCamera();
        virtual void configure();

        osg::Camera *createRTTCam(osg::Camera::BufferComponent buffer, osg::Texture* tex);
        osg::Camera *createHUDCam(double left, double right, double bottom, double top);
        osg::Geode  *createHUDQuad( float width, float height, float scale = 1.0f );

        bool m_configured;
        osg::observer_ptr<osgViewer::View> m_view;
        osg::observer_ptr<osg::Camera> m_l_rtt, m_r_rtt;
        OculusDevice *m_dev;
};

#endif /* HMDCAMERA_H_ */
