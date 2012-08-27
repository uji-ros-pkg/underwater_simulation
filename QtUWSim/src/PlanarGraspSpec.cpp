#include "PlanarGraspSpec.h"
#include <osg/LineWidth>
#include <osg/Point>
#include <osg/AutoTransform>
#include <osgManipulator/AntiSquish>

#include <UWSimUtils.h>


osgManipulator::Dragger* PlanarGraspSpec::createTemplateDragger()
{
	osgManipulator::CustomTabPlaneTrackballDragger *tpt_dragger=new osgManipulator::CustomTabPlaneTrackballDragger();
	tpt_dragger->setupDefaultGeometry();
	return tpt_dragger;
}

osg::Node* PlanarGraspSpec::addDraggerToScene(osg::Node* scene, osgManipulator::Dragger *dragger)
{
    scene->getOrCreateStateSet()->setMode(GL_NORMALIZE, osg::StateAttribute::ON);

    osg::MatrixTransform* selection = new osg::MatrixTransform;
    selection->addChild(scene);

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

PlanarGraspSpec::PlanarGraspSpec(std::string name, osg::Group* group): name_(name) {
	root=group;

	//Create the template bounding box using osg draggers
	t_geode = new osg::Geode;
	t_transform = new osg::MatrixTransform; //TODO: Place it in the center of the view (z of the mosaic; x,y of camera)
	osg::Matrixd transform;
	transform.setRotate(osg::Quat(M_PI_2, osg::Vec3d(1,0,0)));
	transform.setTrans(/*group->getBound().center()+*/osg::Vec3d(0,0,0.01));
	t_transform->setMatrix(osg::Matrixd(transform));
	t_dragger_=createTemplateDragger();
	t_transform.get()->addChild(addDraggerToScene(t_geode.get(), t_dragger_));

	root->addChild(t_transform.get());
}

osg::Geode* PlanarGraspSpec::GraspDragger::createGraspDraggerGeometry() {
	//Create  geometry for the grasp dragger end
	osg::Vec3Array* vertices = new osg::Vec3Array(3);
	(*vertices)[0] = osg::Vec3(-0.04, 0, -0.04);
	(*vertices)[1] = osg::Vec3(0.04, 0, 0);
	(*vertices)[2] = osg::Vec3(-0.04, 0, 0.04);

	osg::Vec4Array* color= new osg::Vec4Array(1);
	(*color)[0] = osg::Vec4f(0, 0, 1.0, 1.0);

	osg::Geometry* geometry = new osg::Geometry();
	geometry->setVertexArray(vertices);
	geometry->setColorArray(color);
	geometry->setColorBinding(osg::Geometry::BIND_OVERALL);
	geometry->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::TRIANGLES,0,vertices->size()));

	osg::Geode* geode = new osg::Geode;
	geode->setName("Grasp Dragger End");
	geode->addDrawable(geometry);

	geode->getOrCreateStateSet()->setMode(GL_LIGHTING,osg::StateAttribute::OFF);
	return geode;
}

osg::Geode* PlanarGraspSpec::GraspDragger::createGraspLineGeometry() {
	//Create  geometry for the grasp line that joins the grasp ends
	l_vertices = new osg::Vec3Array(2);
	(*l_vertices)[0] = tdragger1_->getMatrix().getTrans();
	(*l_vertices)[1] = tdragger2_->getMatrix().getTrans();

	osg::Vec4Array* color= new osg::Vec4Array(1);
	(*color)[0] = osg::Vec4f(1.0, 0, 0.0, 1.0);

	l_geometry = new osg::Geometry();
	l_geometry->setVertexArray(l_vertices);
	l_geometry->setColorArray(color);
	l_geometry->setColorBinding(osg::Geometry::BIND_OVERALL);
	l_prset=new osg::DrawArrays(osg::PrimitiveSet::LINES,0,l_vertices->size());
	l_geometry->addPrimitiveSet(l_prset);

	osg::Geode* geode = new osg::Geode;
	geode->setName("Grasp Dragger Line");
	geode->addDrawable(l_geometry);

	geode->getOrCreateStateSet()->setMode(GL_LIGHTING,osg::StateAttribute::OFF);
	osg::LineWidth* linewidth = new osg::LineWidth();
	linewidth->setWidth(4.0f);
	geode->getOrCreateStateSet()->setAttributeAndModes(linewidth,osg::StateAttribute::ON);

	return geode;
}

PlanarGraspSpec::GraspDragger::GraspDragger() : CompositeDragger() {
	tdragger1_=new osgManipulator::Translate2DDragger();
	tdragger1_->addChild(createGraspDraggerGeometry());

	tdragger2_=new osgManipulator::Translate2DDragger();
	tdragger2_->addChild(createGraspDraggerGeometry());

//    float pixelSize = 50.0f;
//    osg::MatrixTransform* scaler = new osg::MatrixTransform;
//    scaler->setMatrix(osg::Matrix::scale(pixelSize, pixelSize, pixelSize));
//
//    osg::AutoTransform *at = new osg::AutoTransform;
//    at->setAutoScaleToScreen(true);
//    at->addChild(scaler);
//    scaler->addChild(tdragger1_);

	osgManipulator::AntiSquish* as = new osgManipulator::AntiSquish;
	as->addChild(tdragger1_);

	osg::Matrixd local_transform;
	local_transform.setTrans(-0.8,0,0.05);
	local_transform.setRotate(osg::Quat(0, osg::Vec3d(0,1,0)));
	tdragger1_->setMatrix(local_transform);
	local_transform.setTrans(0.8,0,0.05);
	local_transform.setRotate(osg::Quat(M_PI, osg::Vec3d(0,1,0)));
	tdragger2_->setMatrix(local_transform);
	addChild(as);
	addDragger(tdragger1_);
	addChild(tdragger2_);
	addDragger(tdragger2_);

	addChild(createGraspLineGeometry());

    setHandleEvents(true);
	setActivationModKeyMask(osgGA::GUIEventAdapter::MODKEY_CTRL);
}

void PlanarGraspSpec::createGraspDragger() {
	g_dragger_=new GraspDragger();
	((osgManipulator::CustomTabPlaneTrackballDragger*)t_dragger_)->addDragger(g_dragger_);
	((osgManipulator::CustomTabPlaneTrackballDragger*)t_dragger_)->addChild(g_dragger_);
}
