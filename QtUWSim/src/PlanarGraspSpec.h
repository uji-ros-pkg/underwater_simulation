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
	bool axis;
	osg::Geode* axis1, *axis2;
	std::vector<osg::Geode*> vecLines;
};

#endif
