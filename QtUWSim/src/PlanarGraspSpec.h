#ifndef PLANAR_GRASP_SPEC_H
#define PLANAR_GRASP_SPEC_H

#include <osgViewer/ViewerEventHandlers>
#include <osgManipulator/TabPlaneDragger>
#include "CustomTabPlaneTrackballDragger.h"

#include <osg/io_utils>

#include <iostream>

/** This class represents a 2D grasp composed of a bounding box (template) and a pair of antipodal grasps */
class PlanarGraspSpec
{
	/** A custom, initially empty CompositeDragger */
	class CustomCompositeDragger: public osgManipulator::CompositeDragger {
	public:
		CustomCompositeDragger() : CompositeDragger() {}

		~CustomCompositeDragger() {}
	};

	/** A dragger for setting an antipodal grasp */
	class GraspDragger: public osgManipulator::CompositeDragger {
		osgManipulator::Translate2DDragger *tdragger1_;
		osgManipulator::Translate2DDragger *tdragger2_;

		osg::Vec3Array* l_vertices; //Grasp line vertex info
		osg::Geometry* l_geometry; //Grasp line geometry
		osg::PrimitiveSet *l_prset; //Grasp line primitive set

		osg::Geode* createGraspDraggerGeometry();
		osg::Geode* createGraspLineGeometry();

	public:
		GraspDragger();

		virtual bool handle(const osgManipulator::PointerInfo& pi, const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa) {
			tdragger1_->handle(pi, ea, aa);
			tdragger2_->handle(pi, ea, aa);

			//Update line geometry
			(*l_vertices)[0] = tdragger1_->getMatrix().getTrans();
			(*l_vertices)[1] = tdragger2_->getMatrix().getTrans();

			double angle=atan2((*l_vertices)[1].z()-(*l_vertices)[0].z(), (*l_vertices)[1].x()-(*l_vertices)[0].x());
			std::cerr << "dragger1_:" << (*l_vertices)[0] << std::endl;
			std::cerr << "dragger2_:" << (*l_vertices)[1] << std::endl;
			std::cerr << "angle:" << angle  << std::endl;

			osg::Matrixd d1m=tdragger1_->getMatrix();
			osg::Matrixd d2m=tdragger2_->getMatrix();
			d1m.setRotate(osg::Quat(-angle, osg::Vec3d(0,1,0)));
			d2m.setRotate(osg::Quat(M_PI-angle, osg::Vec3d(0,1,0)));
			tdragger1_->setMatrix(d1m);
			tdragger2_->setMatrix(d2m);

			l_geometry->setVertexArray(l_vertices);

			return true;
		}

		~GraspDragger() {}
	};

public:
	PlanarGraspSpec(std::string name, osg::Group*);

	std::string getName() {return name_;}
	
	void setSelected() {setContourColor(osg::Vec4d(1,0,0,1));}
	void setUnselected() {setContourColor(osg::Vec4d(0,1,0,1));}

	void setContourColor(osg::Vec4d color) {
		if (((osgManipulator::CustomTabPlaneTrackballDragger*)t_dragger_)->_tabPlaneDragger) {
			((osgManipulator::CustomTabPlaneTrackballDragger*)t_dragger_)->_tabPlaneDragger->setPlaneColor(color);
		}
	}

	/** Sets the origin of the template */
	void setTemplateOrigin(osg::Vec3d position) {
		osg::Matrixd current=t_transform->getMatrix();
		current.setTrans(position);
		t_transform->setMatrix(current);
	}

	/** Gets the origin of the template */
	osg::Vec3d getTemplateOrigin() {
		return t_transform->getMatrix().getTrans();
	}

	/** Sets the scale of the template */
	void setTemplateScale(float scale) {
		t_dragger_->setMatrix(osg::Matrix::scale(scale, scale, scale) *
		                       osg::Matrix::translate(0,0,0));
	}

	/** Gets the scale of the template */
	osg::Vec3d getTemplateScale() {
		return t_dragger_->getMatrix().getScale();
	}

	void createGraspDragger();

    ~PlanarGraspSpec() {
    	root->removeChild((osg::Node*)t_transform.get());
    }

private:
	std::string name_;
	osg::Group* root;

	//template bounding box and draggers
    osg::ref_ptr<osg::Geode> t_geode;
    osg::ref_ptr<osg::MatrixTransform> t_transform;
    osgManipulator::Dragger* t_dragger_; //Main composite dragger for the template

    GraspDragger *g_dragger_;

    osg::Node* addDraggerToScene(osg::Node* scene, osgManipulator::Dragger *dragger);
    osgManipulator::Dragger* createTemplateDragger();
};

#endif
