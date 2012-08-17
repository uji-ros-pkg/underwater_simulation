#include "PlanarGraspSpec.h"
#include <osg/LineWidth>
#include <osg/Point>

#include <osgManipulator/TabBoxDragger>
#include <osgManipulator/TabBoxTrackballDragger>
#include <osgManipulator/TabPlaneDragger>
#include <osgManipulator/TabPlaneTrackballDragger>
#include <osgManipulator/TrackballDragger>
#include <osgManipulator/Translate1DDragger>
#include <osgManipulator/Translate2DDragger>
#include <osgManipulator/TranslateAxisDragger>
#include <osgManipulator/RotateSphereDragger>

#include <UWSimUtils.h>

/** A custom, initially empty CompositeDragger */
class CustomCompositeDragger: public osgManipulator::CompositeDragger {
public:
        CustomCompositeDragger() : CompositeDragger() {}

        ~CustomCompositeDragger() {}
};


/** A custom RotateCylinder that removes material state set when released */
class CustomRotateCylinderDragger: public osgManipulator::RotateCylinderDragger {

public:
        CustomRotateCylinderDragger() {
                //Modify the default rotation axis
                osg::Cylinder *c=new osg::Cylinder();
                c->setRotation(osg::Quat(M_PI_2,osg::Vec3d(1,0,0)));
                _projector = new osgManipulator::CylinderPlaneProjector(c);
        }

//        bool handle (const osgManipulator::PointerInfo &pi, const osgGA::GUIEventAdapter &ea, osgGA::GUIActionAdapter &us);

        ~CustomRotateCylinderDragger() {}
};


osgManipulator::Dragger* createDragger(const std::string& name)
{

	CustomCompositeDragger *cd=new CustomCompositeDragger();

	osgManipulator::RotateCylinderDragger *rotate_dragger=new osgManipulator::RotateCylinderDragger();

	//Create  geometry for rotate dragger
    osg::Vec3Array* vertices = new osg::Vec3Array(4);
    (*vertices)[0] = osg::Vec3(0.6,-0.1,0);
    (*vertices)[1] = osg::Vec3(0.6,0.1,0);
    (*vertices)[2] = osg::Vec3(0.65,0.1,0);
    (*vertices)[3] = osg::Vec3(0.65,-0.1,0);

    osg::Geometry* geometry = new osg::Geometry();
    geometry->setVertexArray(vertices);
    geometry->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::QUADS,0,vertices->size()));

    osg::Geode* geode = new osg::Geode;
    geode->setName("Dragger Handle");
    geode->addDrawable(geometry);

    geode->getOrCreateStateSet()->setMode(GL_LIGHTING,osg::StateAttribute::OFF);
    osg::LineWidth* linewidth = new osg::LineWidth();
    linewidth->setWidth(5.0f);
    geode->getOrCreateStateSet()->setAttributeAndModes(linewidth,osg::StateAttribute::ON);


	rotate_dragger->addChild(geode);
	osg::Matrixd scalem=rotate_dragger->getMatrix();
	scalem.setRotate(osg::Quat(M_PI_2, osg::Vec3d(1,0,0)));
	rotate_dragger->setMatrix(scalem);
	cd->addChild(rotate_dragger);
	cd->addDragger(rotate_dragger);


	osgManipulator::TabPlaneDragger *translate_dragger=new osgManipulator::TabPlaneDragger();
	translate_dragger->setupDefaultGeometry();
	cd->addChild(translate_dragger);
	cd->addDragger(translate_dragger);

	cd->setParentDragger(cd->getParentDragger());

	return cd;
}

osg::Node* addDraggerToScene(osg::Node* scene)
{
    scene->getOrCreateStateSet()->setMode(GL_NORMALIZE, osg::StateAttribute::ON);

    osg::MatrixTransform* selection = new osg::MatrixTransform;
    selection->addChild(scene);

    osgManipulator::Dragger* dragger = createDragger("PlanarGraspSpec Dragger");

    osg::Group* root = new osg::Group;
    root->addChild(selection);
    root->addChild(dragger);

    float scale = scene->getBound().radius() * 1.6;
    dragger->setMatrix(osg::Matrix::scale(scale, scale, scale) *
                       osg::Matrix::translate(scene->getBound().center()));

    dragger->addTransformUpdating(selection);

    // we want the dragger to handle it's own events automatically
    dragger->setHandleEvents(true);

    // if we don't set an activation key or mod mask then any mouse click on
    // the dragger will activate it, however if do define either of ActivationModKeyMask or
    // and ActivationKeyEvent then you'll have to press either than mod key or the specified key to
    // be able to activate the dragger when you mouse click on it.  Please note the follow allows
    // activation if either the ctrl key or the 'a' key is pressed and held down.
    dragger->setActivationModKeyMask(osgGA::GUIEventAdapter::MODKEY_CTRL);
    dragger->setActivationKeyEvent('a');

    return root;
}

PlanarGraspSpec::PlanarGraspSpec(osg::Group* group){
	root=group;
	axis=false;

	//Create the template bounding box using osg draggers
	t_geode = new osg::Geode;
	t_transform = new osg::MatrixTransform;
	osg::Matrixd transform;
	transform.setRotate(osg::Quat(M_PI_2, osg::Vec3d(1,0,0)));
	transform.setTrans(0,0,0.01);
	t_transform->setMatrix(osg::Matrixd(transform));
	t_transform.get()->addChild(addDraggerToScene(t_geode.get()));

	root->addChild(t_transform.get());
}

void PlanarGraspSpec::addLine(osg::Vec3 point1, osg::Vec3 point2){
	//TODO: create a line between point1 and point2, and embed it into a dragger that allows setting the
	//      start and ending points

	/*
	osg::Geode *geode=new osg::Geode;
	osg::Geometry *g=new osg::Geometry;
	osg::ref_ptr<osg::Vec3Array> points (new osg::Vec3Array());
	osg::ref_ptr<osg::Vec4Array> color (new osg::Vec4Array());


	points->push_back(point1);
	points->push_back(point2);

	color->push_back(osg::Vec4(1.0,1.0,0.0,1.0));
	g->setVertexArray(points.get());
	g->setColorArray(color.get());
	g->setColorBinding(osg::Geometry::BIND_OVERALL);
	g->addPrimitiveSet(new osg::DrawArrays(GL_LINES,0,points->size()));

	osg::LineWidth* linewidth = new osg::LineWidth();
	linewidth->setWidth(4.0f);
	geode->addDrawable( g);
	geode->getOrCreateStateSet()->setAttributeAndModes(linewidth,osg::StateAttribute::ON);
	geode->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);

	root->addChild(geode);
	vecLines.push_back(geode);
	*/
}
void PlanarGraspSpec::addPoint(osg::Vec3 center){
}

void PlanarGraspSpec::drawAxis(osg::Vec3 center, osg::Vec3 pointer){
	//TODO: Create a UWSimGeometry::frame and embed it in a rotatecylinder dragger

	/*
	if(axis){
		root->removeChild(axis1);
		root->removeChild(axis2);
	}
	osg::Geometry *g=new osg::Geometry;
	osg::Geometry *g2=new osg::Geometry;
	osg::ref_ptr<osg::Vec3Array> points (new osg::Vec3Array());
	osg::ref_ptr<osg::Vec4Array> color (new osg::Vec4Array());
	osg::ref_ptr<osg::Vec3Array> points2 (new osg::Vec3Array());
	osg::ref_ptr<osg::Vec4Array> color2 (new osg::Vec4Array());

	osg::Vec3 unitary=osg::Vec3((pointer[0]-center[0])/(sqrt(pow(pointer[0]-center[0],2)+pow(pointer[1]-center[1],2))),
			(pointer[1]-center[1])/(sqrt(pow(pointer[0]-center[0],2)+pow(pointer[1]-center[1],2))), 1.1);




	osg::Vec3 point=osg::Vec3(center[0]+unitary[0]*5, center[1]+unitary[1]*5,1.1);
	osg::Vec3 point2=osg::Vec3(center[0]-unitary[1]*5, center[1]+unitary[0]*5,1.1);


	points->push_back(center);
	points->push_back(point);
	points2->push_back(center);
	points2->push_back(point2);

	color->push_back(osg::Vec4(1.0,0.0,0.0,1.0));
	color2->push_back(osg::Vec4(0.0,1.0,0.0,1.0));
	g->setVertexArray(points.get());
	g->setColorArray(color.get());
	g->setColorBinding(osg::Geometry::BIND_OVERALL);
	g->addPrimitiveSet(new osg::DrawArrays(GL_LINES,0,points->size()));
	axis1=new osg::Geode();
	axis1->addDrawable( g);
	axis1->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
	root->addChild(axis1);
	g2->setVertexArray(points2.get());
	g2->setColorArray(color2.get());
	g2->setColorBinding(osg::Geometry::BIND_OVERALL);
	g2->addPrimitiveSet(new osg::DrawArrays(GL_LINES,0,points2->size()));
	axis2=new osg::Geode();
	axis2->addDrawable( g2);
	axis2->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
	root->addChild(axis2);
	axis=true;
*/
}
void PlanarGraspSpec::deleteLines(){
	/*
	axis=false;
	root->removeChild(axis1);
	root->removeChild(axis2);
	for(int i=0; i<vecLines.size();i++){
		root->removeChild(vecLines[i]);
	}
	vecLines.clear();
*/
}
