#ifndef PLANAR_GRASP_SPEC_H
#define PLANAR_GRASP_SPEC_H
#include <osgViewer/ViewerEventHandlers>




class PlanarGraspSpec
{

public:
	PlanarGraspSpec(osg::Group*);
	void addLine(osg::Vec3, osg::Vec3);
	void addPoint(osg::Vec3);
	void drawAxis(osg::Vec3, osg::Vec3);
	void deleteLines();
	
private:
	osg::Group* root;

	//TODO:
	bool axis;
	osg::Geode* axis1, *axis2;

	//template bounding box
    osg::ref_ptr<osg::Geode> t_geode;
    osg::ref_ptr<osg::MatrixTransform> t_transform;
};

#endif
