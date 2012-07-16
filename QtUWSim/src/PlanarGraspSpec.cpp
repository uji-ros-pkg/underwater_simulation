#include "PlanarGraspSpec.h"


PlanarGraspSpec::PlanarGraspSpec(osg::Group* group){

	root=group;
	axis=false;
}

void PlanarGraspSpec::addLine(osg::Vec3 point1, osg::Vec3 point2){
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

  geode->addDrawable( g); 
  geode->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF); 
    root->addChild(geode);
    vecLines.push_back(geode);
}
void PlanarGraspSpec::addPoint(osg::Vec3 center){
  osg::Geode *geode=new osg::Geode;
  osg::Geometry *g=new osg::Geometry;
  osg::ref_ptr<osg::Vec3Array> points (new osg::Vec3Array());
  osg::ref_ptr<osg::Vec4Array> color (new osg::Vec4Array());
  
  
  points->push_back(osg::Vec3(center[0]-1,center[1],center[2])); 
  points->push_back(osg::Vec3(center[0]+1,center[1],center[2]));
   
  points->push_back(osg::Vec3(center[0],center[1]-1,center[2]));
  points->push_back(osg::Vec3(center[0],center[1]+1,center[2]));  

  color->push_back(osg::Vec4(1.0,1.0,0.0,1.0));
  g->setVertexArray(points.get());
  g->setColorArray(color.get());
  g->setColorBinding(osg::Geometry::BIND_OVERALL);
  g->addPrimitiveSet(new osg::DrawArrays(GL_LINES,0,points->size()));

  geode->addDrawable( g); 
  geode->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF); 
    root->addChild(geode);
    vecLines.push_back(geode);
	
}
void PlanarGraspSpec::drawAxis(osg::Vec3 center, osg::Vec3 pointer){
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
	
}
void PlanarGraspSpec::deleteLines(){
	axis=false;
	root->removeChild(axis1);
	root->removeChild(axis2);
	for(int i=0; i<vecLines.size();i++){
		root->removeChild(vecLines[i]);
	}
	vecLines.clear();
	
}
