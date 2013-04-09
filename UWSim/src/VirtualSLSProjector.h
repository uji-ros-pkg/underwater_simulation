/*
 * VirtualStructuredLightProjector.h
 *
 *  Created on: 06/02/2013
 *      Author: Miquel Massot
 *
 */

#ifndef VirtualSLSProjector_H
#define VirtualSLSProjector_H

#include <osgViewer/Viewer>
#include <osg/Camera>
#include <osg/Texture2D>
#include <osg/Geometry>
#include <osg/Geode>
#include <osg/TexGenNode>
#include <osgDB/Registry>
#include <osgDB/ReadFile>

/**  Virtual range sensor that ... */
class VirtualSLSProjector
{
public:
    std::string name;
    std::string image_name;
    osg::ref_ptr<osg::Node> node;
    osg::ref_ptr<osg::Node> root;
    double range;	///< Max projection range //TODO
    double fov;	    ///< Field of view
    bool visible;	///< Whether to make the beam visible or not
    unsigned int lightNum;
    unsigned int textureUnit;

    VirtualSLSProjector(std::string name, osg::Node *root, osg::Node *node, std::string image_name, double fov, bool visible);
    VirtualSLSProjector();

    void project_on(osg::Node* canvas);
    virtual void init(std::string name, osg::Node *root, osg::Node *node, std::string image_name, double range, double fov, bool visible);
    osg::Node* createSLNode(const osg::Vec3& position, const osg::Vec3& direction, float angle, unsigned int lightNum, unsigned int textureUnit);
	osg::StateSet* createSLDecoratorState(osg::StateSet* stateset, unsigned int lightNum, unsigned int textureUnit);
};

#endif
