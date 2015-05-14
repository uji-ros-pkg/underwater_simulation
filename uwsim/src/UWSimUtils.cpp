/* 
 * Copyright (c) 2013 University of Jaume-I.
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the GNU Public License v3.0
 * which accompanies this distribution, and is available at
 * http://www.gnu.org/licenses/gpl.html
 * 
 * Contributors:
 *     Mario Prats
 *     Javier Perez
 */

#include <ros/ros.h>
#include <uwsim/SimulatorConfig.h>
#include <uwsim/UWSimUtils.h>
#include <osg/Material>
#include <osg/ShapeDrawable>
#include <osg/Shape>
#include <osg/Geode>
#include <osg/Switch>
#include <osg/MatrixTransform>
#include <osgDB/ReadFile>

#if OSG_VERSION_MAJOR>=3
#include <osgDB/Options>
#endif


// Default constructor - initialize searchForName to "" and 
// set the traversal mode to TRAVERSE_ALL_CHILDREN
findNodeVisitor::findNodeVisitor() :
    osg::NodeVisitor(TRAVERSE_ALL_CHILDREN), searchForName()
{
}

// Constructor that accepts string argument
// Initializes searchForName to user string
// set the traversal mode to TRAVERSE_ALL_CHILDREN
findNodeVisitor::findNodeVisitor(const std::string &searchName) :
    osg::NodeVisitor(TRAVERSE_ALL_CHILDREN), searchForName(searchName)
{
}

//The 'apply' method for 'node' type instances.
//Compare the 'searchForName' data member against the node's name.
//If the strings match, add this node to our list
void findNodeVisitor::apply(osg::Node &searchNode)
{
  //std::cerr << "Compare " << searchForName << " to "  << searchNode.getName() << std::endl;
  if (searchNode.getName() == searchForName)
  {
    foundNodeList.push_back(&searchNode);
  }
  traverse(searchNode);
}

// Set the searchForName to user-defined string
void findNodeVisitor::setNameToFind(const std::string &searchName)
{
  searchForName = searchName;
  foundNodeList.clear();
}

osg::Node* findNodeVisitor::getFirst()
{
  if (foundNodeList.size() == 0)
    return NULL;
  else
    return foundNodeList[0];
}

findRoutedNode::findRoutedNode() :
    nodeVisitor()
{
}

findRoutedNode::findRoutedNode(const std::string &searchName) :
    nodeVisitor(), searchRoute(searchName)
{
}

void findRoutedNode::setNameToFind(const std::string &searchName)
{
  searchRoute = searchName;
  rootList.clear();
}

void findRoutedNode::find(osg::ref_ptr<osg::Node> searchNode)
{
  unsigned int pos = 0;
  rootList.clear();
  rootList.push_back(searchNode);
  nodeListType auxList, auxList2;

  while ((pos = searchRoute.find("/")) < searchRoute.size())
  {
    for (unsigned int i = 0; i < rootList.size(); i++)
    {
      nodeVisitor.setNameToFind(searchRoute.substr(0, pos));
      rootList[i]->accept(nodeVisitor);
      auxList2 = nodeVisitor.getNodeList();
      auxList.insert(auxList.end(), auxList2.begin(), auxList2.end());
    }
    searchRoute.erase(0, pos + 1);
    rootList = auxList;
    auxList.clear();
  }
  for (unsigned int i = 0; i < rootList.size(); i++)
  {
    nodeVisitor.setNameToFind(searchRoute);
    rootList[i]->accept(nodeVisitor);
    auxList2 = nodeVisitor.getNodeList();
    auxList.insert(auxList.end(), auxList2.begin(), auxList2.end());
  }
  rootList = auxList;
}

osg::Node* findRoutedNode::getFirst()
{
  if (rootList.size() == 0)
    return NULL;
  else
    return rootList[0];
}

osg::Node * findRN(std::string target, osg::Group * root)
{
  findRoutedNode findRN(target);
  findRN.find(root);
  return findRN.getFirst();
}

//Default mask is for AR objects (not shown on Virtual Cameras)
osg::Node* UWSimGeometry::createSwitchableFrame(double radius, double length, unsigned int mask)
{
  osg::Switch *axis = new osg::Switch();
  axis->setNewChildDefaultValue(false);
  axis->setName("switch_frames");
  axis->addChild(UWSimGeometry::createFrame());
  axis->setNodeMask(mask);
  return axis;
}

osg::Node* UWSimGeometry::createFrame(double radius, double length)
{
  osg::Matrix linkBaseMatrix;
  linkBaseMatrix.makeIdentity();
  osg::MatrixTransform *linkBaseTransform = new osg::MatrixTransform(linkBaseMatrix);

  //create XBase to rotate
  osg::Matrix XBase;
  XBase.makeIdentity();
  XBase.preMultRotate(osg::Quat(M_PI_2, osg::Vec3d(0, 1, 0)));
  XBase.preMultTranslate(osg::Vec3d(0, 0, length / 2));
  osg::MatrixTransform *XBaseTransform = new osg::MatrixTransform(XBase);
  linkBaseTransform->addChild(XBaseTransform);

  //create X cylinder, set color, and add to XBase
  osg::Node * Xcylinder = UWSimGeometry::createOSGCylinder(radius, length);
  osg::StateSet * Xstateset = new osg::StateSet();
  osg::Material * Xmaterial = new osg::Material();
  Xmaterial->setDiffuse(osg::Material::FRONT_AND_BACK, osg::Vec4(1, 0, 0, 0));
  Xstateset->setAttribute(Xmaterial);
  Xcylinder->setStateSet(Xstateset);
  XBaseTransform->addChild(Xcylinder);

  //create YBase to rotate
  osg::Matrix YBase;
  YBase.preMultRotate(osg::Quat(M_PI_2, osg::Vec3d(1, 0, 0)));
  YBase.preMultTranslate(osg::Vec3d(0, 0, -length / 2));
  osg::MatrixTransform *YBaseTransform = new osg::MatrixTransform(YBase);
  linkBaseTransform->addChild(YBaseTransform);

  //create Y cylinder, set color, and add to YBase
  osg::Node *Ycylinder = UWSimGeometry::createOSGCylinder(radius, length);
  osg::StateSet * Ystateset = new osg::StateSet();
  osg::Material * Ymaterial = new osg::Material();
  Ymaterial->setDiffuse(osg::Material::FRONT_AND_BACK, osg::Vec4(0, 1, 0, 0));
  Ystateset->setAttribute(Ymaterial);
  Ycylinder->setStateSet(Ystateset);
  YBaseTransform->addChild(Ycylinder);

  //create ZBase to rotate
  osg::Matrix ZBase;
  ZBase.makeIdentity();
  ZBase.preMultRotate(osg::Quat(M_PI_2, osg::Vec3d(0, 0, 1)));
  ZBase.preMultTranslate(osg::Vec3d(0, 0, length / 2));
  osg::MatrixTransform *ZBaseTransform = new osg::MatrixTransform(ZBase);
  linkBaseTransform->addChild(ZBaseTransform);

  //create Z cylinder, set color, and add to ZBase
  osg::Node * Zcylinder = UWSimGeometry::createOSGCylinder(radius, length);
  osg::StateSet * Zstateset = new osg::StateSet();
  osg::Material * Zmaterial = new osg::Material();
  Zmaterial->setDiffuse(osg::Material::FRONT_AND_BACK, osg::Vec4(0, 0, 1, 0));
  Zstateset->setAttribute(Zmaterial);
  Zcylinder->setStateSet(Zstateset);
  ZBaseTransform->addChild(Zcylinder);

  return linkBaseTransform;
}

osg::Node * UWSimGeometry::createOSGBox(osg::Vec3 size)
{
  osg::Box *box = new osg::Box();

  box->setHalfLengths(size / 2);

  osg::ShapeDrawable *shape = new osg::ShapeDrawable(box);
  osg::Geode *geode = new osg::Geode();
  geode->addDrawable(shape);

  osg::Node* node = new osg::Group();
  node->asGroup()->addChild(geode);

  return node;
}

osg::Node* UWSimGeometry::createOSGCylinder(double radius, double height)
{
  osg::Cylinder *cylinder = new osg::Cylinder();

  cylinder->setRadius(radius);
  cylinder->setHeight(height);

  osg::ShapeDrawable *shape = new osg::ShapeDrawable(cylinder);
  osg::Geode *geode = new osg::Geode();
  geode->addDrawable(shape);

  osg::Node *node = new osg::Group();
  node->asGroup()->addChild(geode);

  return node;
}

osg::Node * UWSimGeometry::createOSGSphere(double radius)
{
  osg::Sphere *sphere = new osg::Sphere();

  sphere->setRadius(radius);

  osg::ShapeDrawable *shape = new osg::ShapeDrawable(sphere);
  osg::Geode *geode = new osg::Geode();
  geode->addDrawable(shape);

  osg::Node *node = new osg::Group();
  node->asGroup()->addChild(geode);

  return node;
}

osg::Node * UWSimGeometry::createLabel(std::string textToDraw,double charSize, int bb, osg::Vec4 color )
{
  //Create text
  osg::ref_ptr<osgText::Text> text = new osgText::Text;
  text->setFont( "fonts/arial.ttf" );
  text->setText(textToDraw);
  text->setAxisAlignment( osgText::TextBase::SCREEN );
  text->setDataVariance( osg::Object::DYNAMIC );
  text->setColor(color);
  text->setCharacterSize(charSize);
  if(bb)
  {
    text->setBoundingBoxColor(color);
    text->setDrawMode(osgText::Text::TEXT | osgText::Text::ALIGNMENT | osgText::Text::BOUNDINGBOX);
  }
  else
    text->setDrawMode(osgText::Text::TEXT | osgText::Text::ALIGNMENT);

  //set visual properties
  osg::ref_ptr<osg::Geode> geode = new osg::Geode;
  geode->addDrawable( text.get() );
  geode->getOrCreateStateSet()->setMode(GL_DEPTH_TEST, osg::StateAttribute::OFF); //Draw it over geometry
  geode->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF );  //Ignore shadows

  geode->getOrCreateStateSet()->setAttributeAndModes(new osg::Program(), osg::StateAttribute::ON); //Unset shader
  return geode.release();
}

osg::Node * UWSimGeometry::retrieveResource(std::string name)
{
  //Load file in memory
  resource_retriever::Retriever r;
  resource_retriever::MemoryResource resource;

  try
  {
    resource = r.get(name);
  }
  catch (resource_retriever::Exception& e)
  {
    return NULL;
  }

  //Create stream with memory resource
  std::stringstream buffer;
  buffer.write((char *)resource.data.get(), resource.size);

  //Get file extension and create options
  std::string file_ext = osgDB::getFileExtension(name);
#if OSG_VERSION_MAJOR>=3
  osg::ref_ptr<osgDB::Options> local_opt = new osgDB::Options;
#endif

  //Check if file format is supported, get readerwriter
  osgDB::ReaderWriter* rw = osgDB::Registry::instance()->getReaderWriterForExtension(file_ext);
  if (!rw)
  {
    std::cout << "Data file format " << file_ext << " not supported" << std::endl;
    return NULL;
  }

  //Try loading the resource,
#if OSG_VERSION_MAJOR>=3
  osgDB::ReaderWriter::ReadResult readResult = rw->readNode( buffer,local_opt.get());
#else
  osgDB::ReaderWriter::ReadResult readResult = rw->readNode(buffer);
#endif
  if (readResult.validNode())
    return readResult.takeNode();
  else
    std::cout << "Can't load file " << name << std::endl;
  return NULL;

}

osg::Node * UWSimGeometry::loadGeometry(boost::shared_ptr<Geometry> geom)
{
  if (geom->type == 0)
  {
    osg::Node * node = retrieveResource(geom->file);
    if (node == NULL)
    {
      //retrieve resource didn't succeed, let's search in the DATA PATH
      const std::string SIMULATOR_DATA_PATH = std::string(getenv("HOME")) + "/.uwsim/data";

      osgDB::Registry::instance()->getDataFilePathList().push_back(std::string(SIMULATOR_DATA_PATH));
      osgDB::Registry::instance()->getDataFilePathList().push_back(
          std::string(SIMULATOR_DATA_PATH) + std::string("/objects"));
      osgDB::Registry::instance()->getDataFilePathList().push_back(
          std::string(SIMULATOR_DATA_PATH) + std::string("/terrain"));
      osgDB::Registry::instance()->getDataFilePathList().push_back(
          std::string(UWSIM_ROOT_PATH) + std::string("/data/shaders"));
      node = osgDB::readNodeFile(geom->file);

      if (node == NULL)
      {
        std::cerr << "Error retrieving file " << geom->file
            << " Check URDF file or set your data path with the --dataPath option." << std::endl;
        exit(0);
      }
    }
    //If node isn't a group create a group with it.
    if (node->asGroup() == NULL)
    {
      osg::Node * aux = node;
      node = new osg::Group();
      node->asGroup()->addChild(aux);
    }
    return node;
  }
  else if (geom->type == 1)
  {
    return UWSimGeometry::createOSGBox(osg::Vec3(geom->boxSize[0], geom->boxSize[1], geom->boxSize[2]));
  }
  else if (geom->type == 2)
    return UWSimGeometry::createOSGCylinder(geom->radius, geom->length);
  else if (geom->type == 3)
    return UWSimGeometry::createOSGSphere(geom->radius);
  else if (geom->type == 4)
    return new osg::Group();
  std::cerr << "Unknown geometry type. " << std::endl;
  exit(0);
  return NULL;
}

getWorldCoordOfNodeVisitor::getWorldCoordOfNodeVisitor() :
    osg::NodeVisitor(NodeVisitor::TRAVERSE_PARENTS), done(false)
{
  wcMatrix.reset(new osg::Matrixd());
}

void getWorldCoordOfNodeVisitor::apply(osg::Node &node)
{
  if (!done)
  {
    if (0 == node.getNumParents())
    { // no parents
      wcMatrix->set(osg::computeLocalToWorld(this->getNodePath()));
      done = true;
    }
    traverse(node);
  }
}

boost::shared_ptr<osg::Matrix> getWorldCoordOfNodeVisitor::giveUpDaMat()
{
  return wcMatrix;
}

boost::shared_ptr<osg::Matrix> getWorldCoords(osg::Node* node)
{
  osg::ref_ptr<getWorldCoordOfNodeVisitor> ncv = new getWorldCoordOfNodeVisitor();
  if (node && ncv)
  {
    node->accept(*ncv);
    return ncv->giveUpDaMat();
  }
  else
  {
    return boost::shared_ptr<osg::Matrix>();
  }
}


//Mudded object functions (maybe a namespace is useful)


#include <osg/ComputeBoundsVisitor>

#include <osgParticle/ModularEmitter>
#include <osgParticle/ParticleSystemUpdater>
#include <osgParticle/SectorPlacer>

#include <osgParticle/FluidFrictionOperator>
#include <osgParticle/AccelOperator>
#include <osgParticle/ModularProgram>

#include "uwsim/SimulatedIAUV.h"

class AttractOperator : public osgParticle::Operator
{
  public:
    AttractOperator() : osgParticle::Operator(), _magnitude(1.0f), _ratio(0.5), _killSink(true){}
    AttractOperator( const AttractOperator& copy, const osg::CopyOp& copyop = osg::CopyOp::SHALLOW_COPY )
      : osgParticle::Operator(copy, copyop), _center(copy._center), _magnitude(copy._magnitude), _ratio(copy._ratio), _killSink(copy._killSink)
      {}
    META_Object( spin, AttractOperator );
    /// Set the center of the attractive force
    void setCenter( const osg::Vec3& c ) { _center = c; }
    /// Get the center of the attractive force
    const osg::Vec3& getCenter() const { return _center; }
    /// Set the acceleration scale
    void setMagnitude( float mag ) { _magnitude = mag; }
    /// Get the acceleration scale
    float getMagnitude() const { return _magnitude; }
    /// Set the attraction ratio (CURRENTLY UNUSED)
    void setRatio( float r ) { _ratio = r; if (_ratio<0.0f) _ratio=0.0f; if (_ratio>1.0f) _ratio=1.0f; }
    /// Get the attraction ratio
    float getRatio() const { return _ratio; }
    /// Set whether the attractor kills the particles once they arrive
    void setKillSink( bool kill ) { _killSink = kill; }
    /// Get whether the attractor kills the particles once they arrive
    bool getKillSink() const { return _killSink; }
    /// Apply attraction to a particle. Do not call this method manually.
    inline void operate( osgParticle::Particle* P, double dt );
    /// Perform some initializations. Do not call this method manually.
    inline void beginOperate( osgParticle::Program* prg );

    /// Set the center of the attractive force
    void setNode( osg::Group * root ) { _root = root;}  

    /// Set the rotation of the object
    void setParent( osg::Node * parent ) { _parent = parent;}  

  protected:
    virtual ~AttractOperator() {}
    AttractOperator& operator=( const AttractOperator& ) { return *this; }
    osg::Vec3 _center;
    osg::Vec3 _xf_center;
    osg::Group * _root;
    osg::Node * _parent;
    float _magnitude;
    float _ratio;
    bool _killSink;
};

inline void AttractOperator::operate( osgParticle::Particle* P, double dt )
{
  osg::Vec3 dir = _xf_center - P->getPosition();
  if (dir.length()>0.1 )
  {
    // similar to orbit (but without epsilon):
    //P->addVelocity( dir * _magnitude * dt * (1-_ratio) );
    // close, but changes absolute position when we move center:
    //P->addVelocity( dir * _magnitude * dt * (1-_ratio) );
    //P->setPosition(P->getPosition() + (dir * _magnitude * dt * _ratio));
    // make rotation from current direction to target direction:
    /*osg::Matrix mat;
    osg::Vec3 current = P->getVelocity();
    current.normalize();
    dir.normalize();
    mat.makeRotate(current, dir);
    float scalar_factor = 0.01;
    P->transformPositionVelocity(osg::Matrix::identity(), mat, 1/(1+(_magnitude*_magnitude*0.001)) );*/
    //P->transformPositionVelocity(osg::Matrix::identity(), mat, _ratio);

    //osg::Vec3 target = osg::Vec3(0,0,100)-P->getPosition();
    P->setVelocity(dir*(1/dir.length())*_magnitude);
  }
  else
  {
    if (_killSink)
    {
      P->kill();
    }
    else
    {
      P->setPosition(_xf_center);
      P->setVelocity(osg::Vec3(0,0,0));
    }
  }
}

inline void AttractOperator::beginOperate( osgParticle::Program* prg )
{
  boost::shared_ptr<osg::Matrix> XuponaMat=getWorldCoords(findRN("end_effector",_root));
  boost::shared_ptr<osg::Matrix> _mat=getWorldCoords(_parent);
  _mat->invert(*_mat);

  _center=XuponaMat->getTrans();

  if ( prg->getReferenceFrame()==osgParticle::ModularProgram::RELATIVE_RF )
  {
    _xf_center = ( *XuponaMat  * *_mat).getTrans();
  }
  else
  {
    _xf_center = _center;
  }

  //std::cout<<_rotation.x()<<" "<<_rotation.y()<<" "<<_rotation.z()<<" "<<_rotation.w()<<std::endl;
  //std::cout<<_xf_center.x()<<" "<<_xf_center.y()<<" "<<_xf_center.z()<<std::endl;
}

osgParticle::ParticleSystem* createSmokeParticles( osg::Group* parent, osg::Group * root,double x, double y, osgParticle::RandomRateCounter * rrc)
{
    osg::ref_ptr<osgParticle::ParticleSystem> ps = new osgParticle::ParticleSystem;
    ps->getDefaultParticleTemplate().setLifeTime( 15.0f );
    ps->getDefaultParticleTemplate().setShape( osgParticle::Particle::QUAD );
    ps->getDefaultParticleTemplate().setSizeRange( osgParticle::rangef(0.05f, 0.1f) );
    ps->getDefaultParticleTemplate().setAlphaRange( osgParticle::rangef(1.0f, 0.0f) );
    ps->getDefaultParticleTemplate().setColorRange(
        osgParticle::rangev4(osg::Vec4(0.3f,0.2f,0.01f,1.0f), osg::Vec4(0.15f,0.1f,0.01f,0.5f)) );
    ps->setDefaultAttributes( "smoke.rgb", true, false );
    
    rrc->setRateRange( 200, 500 );
    
    /*osg::ref_ptr<osgParticle::RadialShooter> shooter = new osgParticle::RadialShooter;
    shooter->setThetaRange( -osg::PI_4*0.5f, osg::PI_4*0.5f );
    shooter->setPhiRange( -osg::PI_4*0.5f, osg::PI_4*0.5f );
    shooter->setInitialSpeedRange( 10.0f, 15.0f );*/

    //osg::ref_ptr<CustomShooter> shooter = new CustomShooter;

    osg::ref_ptr<osgParticle::SectorPlacer> placer = new osgParticle::SectorPlacer;
    placer->setRadiusRange(x/2,y/2);
    //placer->setPhiRange(0,  2 * osg::PI);

    
    osg::ref_ptr<osgParticle::ModularEmitter> emitter = new osgParticle::ModularEmitter;
    emitter->setPlacer(placer);
    emitter->setParticleSystem( ps.get() );
    emitter->setCounter( rrc );
    //emitter->setShooter( shooter.get() );
    parent->addChild( emitter.get() );


    osgParticle::ModularProgram *moveDustInAir = new osgParticle::ModularProgram;
    moveDustInAir->setParticleSystem(ps);

    /*osgParticle::AccelOperator *accelUp = new osgParticle::AccelOperator;
    accelUp->setToGravity(-5); // scale factor for normal acceleration due to gravity. 
    moveDustInAir->addOperator(accelUp);*/
    
    AttractOperator * attOp= new AttractOperator;
    attOp->setCenter(osg::Vec3(0,0,1));
    attOp->setNode(root);
    attOp->setParent(parent);
    attOp->setMagnitude(0.5);
    attOp->setRatio(50);
    moveDustInAir->addOperator(attOp);

    osgParticle::FluidFrictionOperator *airFriction = new osgParticle::FluidFrictionOperator;
    airFriction->setFluidToWater();
    moveDustInAir->addOperator(airFriction);

    parent->addChild(moveDustInAir);

    return ps.get();
}


DynamicHF::DynamicHF(osg::HeightField* height, osg::Group * root, boost::shared_ptr<osg::Matrix> mat, std::vector<boost::shared_ptr<AbstractDredgeTool> > tools)
{
  dredgeTools=tools;
  heightField=height;
  this->root=root;
  objectMat=mat;
  mat->preMultRotate(heightField->getRotation());
  emitter=NULL;
  nparticles=0;
}

void DynamicHF::addParticleSystem(osgParticle::RandomRateCounter * rrc){
  emitter=rrc;
}

void DynamicHF::update( osg::NodeVisitor*,osg::Drawable* drawable )
{

  for(unsigned int i=0;i<dredgeTools.size();i++)
  {
    boost::shared_ptr<osg::Matrix> XuponaMat=dredgeTools[i]->getDredgePosition();

  //std::cout<<XuponaMat->getTrans().x()<<" "<<XuponaMat->getTrans().y()<<" "<<XuponaMat->getTrans().z()<<" "<<std::endl;
  //std::cout<<objectMat->getTrans().x()<<" "<<objectMat->getTrans().y()<<" "<<objectMat->getTrans().z()<<" "<<std::endl;
  //std::cout<<"Origin: "<<heightField->getOrigin().x()<<" "<<heightField->getOrigin().y()<<" "<<heightField->getOrigin().z()<<std::endl;

    int modified=0;

    for (int r = 0; r < heightField->getNumRows(); r++) {
      for (int c = 0; c < heightField->getNumColumns(); c++) {
        if( (  XuponaMat->getTrans() - (objectMat->getTrans() + (heightField->getRotation().inverse() *heightField->getOrigin()) + 
          osg::Vec3(c*heightField->getXInterval(),r*heightField->getYInterval(),heightField->getHeight(c,r)))
          ).length2()< 0.01){
           heightField->setHeight(c, r,heightField->getHeight(c,r)-0.01);
           modified=1;
           nparticles++;
        }
      }
    }

    if(modified){
      drawable->dirtyDisplayList();
      drawable->dirtyBound();
    }
    else
      nparticles-=max(nparticles*0.1,0.0);

    if(emitter){
      emitter->setRateRange(min(nparticles*40,1000),min(nparticles*80,2000));
    }
  }
}

osg::Node* createHeightField(osg::ref_ptr<osg::Node> object, std::string texFile, double percent, osg::Group * root, const std::vector<boost::shared_ptr<SimulatedIAUV> >  vehicles)
{
    
  //osg::Image* heightMap = osgDB::readImageFile(heightFile);
  osg::ComputeBoundsVisitor cbv;
  object->accept(cbv);
  osg::BoundingBox box = cbv.getBoundingBox();

  boost::shared_ptr<osg::Matrix> mat=getWorldCoords( object);


  box._min= mat->getRotate() * box._min;
  box._max= mat->getRotate() * box._max;

  //std::cout<<box.xMin()<<" "<<box.yMin()<<" "<<box.zMin()<<std::endl;
  //std::cout<<box.xMax()<<" "<<box.yMax()<<" "<<box.zMax()<<std::endl;


  //Adjust resolution to closest multiple for each axis
  float resX=0.01 + fmod((double)abs(box.xMax()-box.xMin()),0.01) / (double)floor(abs(box.xMax()-box.xMin())/0.01);
  float resY=0.01 + fmod((double)abs(box.yMax()-box.yMin()),0.01) / (double)floor(abs(box.yMax()-box.yMin())/0.01);
 
  //std::cout<<"Resolution: "<<resX<<" "<<resY<<std::endl;
  //std::cout<<"Nelems: "<<(abs(box.xMax()-box.xMin())/(resX)+1)<<" "<<(abs(box.yMax()-box.yMin())/(resY)+1)<<std::endl;

  int addedElems = abs(box.zMax()-box.zMin())*percent / 0.01*3;
     
  osg::HeightField* heightField = new osg::HeightField();
  heightField->allocate(abs(box.xMax()-box.xMin())/(resX)+1+addedElems*2,abs(box.yMax()-box.yMin())/(resY)+1+addedElems*2);
  heightField->setOrigin(mat->getRotate().inverse() * osg::Vec3(min(box.xMin(),box.xMax())-addedElems*resX,min(box.yMin(),box.yMax())-addedElems*resY, min(box.zMin(),box.zMax())));
  heightField->setRotation(mat->getRotate().inverse());  //TODO: does not work with scales!
  heightField->setXInterval(resX);
  heightField->setYInterval(resY);
  heightField->setSkirtHeight(0.01f);

  //std::cout<<"Allocate: "<<(abs(box.xMax()-box.xMin())/(resX)+1)<<" "<<(abs(box.yMax()-box.yMin())/(resY)+1)<<std::endl;        
  //std::cout<<"Origin: "<<heightField->getOrigin().x()<<" "<<heightField->getOrigin().y()<<" "<<heightField->getOrigin().z()<<std::endl; 
  //std::cout<<"Height: "<<(box.zMax()-box.zMin())*percent<<std::endl;

  for (int r = 0; r < heightField->getNumRows(); r++) {
    for (int c = 0; c < heightField->getNumColumns(); c++) {
      heightField->setHeight(c, r, abs(box.zMax()-box.zMin())*percent );
      if(r<addedElems)
       heightField->setHeight(c, r,min((double)heightField->getHeight(c,r), (1 - ( (addedElems-r)*(addedElems-r) / ((double)(addedElems)*(addedElems)))) * abs(box.zMax()-box.zMin())*percent) ); // r*resY) );  
      if(heightField->getNumRows()-r<addedElems)
       heightField->setHeight(c, r,min((double)heightField->getHeight(c,r),(1 - ( (addedElems-(heightField->getNumRows()-r))*(addedElems-(heightField->getNumRows()-r)) / ((double)(addedElems)*(addedElems)))) * abs(box.zMax()-box.zMin())*percent) );      //(heightField->getNumRows()-r)*resY) );  
      if(c<addedElems)
       heightField->setHeight(c, r,min((double)heightField->getHeight(c,r), (1 - ( (addedElems-c)*(addedElems-c) / ((double)(addedElems)*(addedElems)))) * abs(box.zMax()-box.zMin())*percent) ); //c*resX) );  
      if(heightField->getNumColumns()-c<addedElems)
       heightField->setHeight(c, r,min((double)heightField->getHeight(c,r), (1 - ( (addedElems-(heightField->getNumColumns()-c))*(addedElems-(heightField->getNumColumns()-c)) / ((double)(addedElems)*(addedElems)))) * abs(box.zMax()-box.zMin())*percent) ); //(heightField->getNumColumns()-c)*resX) );  

    }
  }

  //Search for dredge tools on vehicles,
  std::vector<boost::shared_ptr<AbstractDredgeTool> > dredgeTools;
  for(unsigned int i=0;i<vehicles.size();i++)
  {
    for(unsigned int j=0;j<vehicles[i]->devices->all.size();j++)
    {
      boost::shared_ptr<AbstractDredgeTool>  dredgeTool = boost::dynamic_pointer_cast < AbstractDredgeTool >  (vehicles[i]->devices->all[j]);

      if(dredgeTool)
        dredgeTools.push_back(dredgeTool);
    }
  }


  osg::Geode* geode = new osg::Geode();
  osg::ShapeDrawable* draw=new osg::ShapeDrawable(heightField);
  geode->addDrawable(draw);
  DynamicHF * dynamicHF=new DynamicHF(heightField, root,mat, dredgeTools );
  draw->setUpdateCallback( dynamicHF);
     
  osg::Texture2D* tex = new osg::Texture2D(osgDB::readImageFile(texFile));
  //tex->setFilter(osg::Texture2D::MIN_FILTER,osg::Texture2D::LINEAR_MIPMAP_LINEAR);
  //tex->setFilter(osg::Texture2D::MAG_FILTER,osg::Texture2D::LINEAR);
  tex->setWrap(osg::Texture::WRAP_S, osg::Texture::REPEAT);
  tex->setWrap(osg::Texture::WRAP_T, osg::Texture::REPEAT);
  geode->getOrCreateStateSet()->setTextureAttributeAndModes(0, tex);

  //PARTICLE SYSTEM

  osg::ref_ptr<osg::MatrixTransform> parent = new osg::MatrixTransform;
  parent->setMatrix( osg::Matrix::rotate(-osg::PI_2, osg::X_AXIS) * osg::Matrix::translate(0.0f,0.0f,0.2f) );

  osgParticle::RandomRateCounter * rrc= new osgParticle::RandomRateCounter;;

  osgParticle::ParticleSystem* smoke = createSmokeParticles(parent,root,heightField->getNumColumns()*resX,heightField->getNumRows()*resY,rrc);

  osg::ref_ptr<osgParticle::ParticleSystemUpdater> updater = new osgParticle::ParticleSystemUpdater;
  updater->addParticleSystem( smoke );
  dynamicHF->addParticleSystem(rrc);
  osg::ref_ptr<osg::Geode> smokeGeode = new osg::Geode;
  smokeGeode->getOrCreateStateSet()->setAttributeAndModes(new osg::Program(), osg::StateAttribute::ON);
  smokeGeode->addDrawable( smoke );

  parent->addChild(updater.get() );
  parent->addChild(smokeGeode.get() );
  object->asGroup()->addChild( parent.get() );
     
  return geode;
}

