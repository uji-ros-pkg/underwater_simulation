#include "URDFRobot.h"
#include "UWSimUtils.h"
#include <osgOcean/ShaderManager>
#include <osg/ShapeDrawable>
#include <osg/Material>
#include <math.h>

URDFRobot::URDFRobot(osgOcean::OceanScene *oscene,Vehicle vehicle): KinematicChain(vehicle.nlinks, vehicle.njoints) {
   ScopedTimer buildSceneTimer("Loading URDF robot... \n", osg::notify(osg::ALWAYS));
   osgDB::Registry::instance()->getDataFilePathList().push_back(std::string(SIMULATOR_DATA_PATH)+std::string("/robot"));
   osgDB::Registry::instance()->getDataFilePathList().push_back(std::string(SIMULATOR_DATA_PATH)+std::string("/shaders"));
   osgDB::Registry::instance()->getDataFilePathList().push_back(std::string(SIMULATOR_DATA_PATH)+std::string("/textures"));

   for(int i=0;i<vehicle.nlinks;i++){
	if(vehicle.links[i].type==0){
	//Find file in the package
  	if(vehicle.links[i].file.rfind("/robot/"))
    	   vehicle.links[i].file.erase(0,vehicle.links[i].file.rfind("/robot/")+7);
  	//Use osg version of file
  	vehicle.links[i].file.erase(vehicle.links[i].file.size()-3);
  	vehicle.links[i].file.append("osg");
	}
   }

   link.resize(vehicle.nlinks);

   for(int i=0; i<vehicle.nlinks;i++) {
	ScopedTimer buildSceneTimer("  · "+vehicle.links[i].file+": ", osg::notify(osg::ALWAYS));
	if(vehicle.links[i].type==0){
	  link[i] = osgDB::readNodeFile(vehicle.links[i].file);
	  if(link[i] == NULL){
	     std::cerr<<"Error reading file " << vehicle.links[i].file <<". Check URDF file." <<std::endl;
	     exit(0);
	  }
        }
	else if(vehicle.links[i].type==1){
	  link[i] = UWSimGeometry::createOSGBox(osg::Vec3(vehicle.links[i].boxSize[0], vehicle.links[i].boxSize[1], vehicle.links[i].boxSize[2]));
	}
	else if(vehicle.links[i].type==2)
	  link[i] = UWSimGeometry::createOSGCylinder(vehicle.links[i].radius,vehicle.links[i].length);
	else if(vehicle.links[i].type==3)
	  link[i] = UWSimGeometry::createOSGSphere(vehicle.links[i].radius);
	link[i]->setName(vehicle.links[i].name);
	if(vehicle.links[i].material!=-1){ //Add material if exists
	  osg::ref_ptr<osg::StateSet> stateset = new osg::StateSet();
	  osg::ref_ptr<osg::Material> material = new osg::Material();
          material->setDiffuse(osg::Material::FRONT_AND_BACK, osg::Vec4(vehicle.materials[vehicle.links[i].material].r, vehicle.materials[vehicle.links[i].material].g, vehicle.materials[vehicle.links[i].material].b, vehicle.materials[vehicle.links[i].material].a));
	  stateset->setAttribute(material);
	  link[i]->setStateSet(stateset);
	}
   }

   bool success=true;
   for (int i=0; i<vehicle.nlinks;i++){
	   if(!link[i].valid()){
		osg::notify(osg::WARN) << "Could not find "<<vehicle.name<< " link" << i << ".osg "<<vehicle.links[i+1].file  << std::endl;
		baseTransform=NULL;
		success=false;
   	   }
   }

   //Create a frame that can be switched on and off 
   osg::ref_ptr<osg::Node> axis=UWSimGeometry::createSwitchableFrame();

   if(success) {
	ScopedTimer buildSceneTimer("  · Linking links...", osg::notify(osg::ALWAYS));
        osgDB::Registry::instance()->getDataFilePathList().push_back(std::string(SIMULATOR_DATA_PATH)+std::string("/shaders"));
	static const char model_vertex[]   = "default_scene.vert";
	static const char model_fragment[] = "default_scene.frag";
	std::vector<osg::ref_ptr<osg::MatrixTransform> > linkBaseTransforms(vehicle.nlinks);
	std::vector<osg::ref_ptr<osg::MatrixTransform> > linkPostTransforms(vehicle.nlinks);

	osg::Matrix linkBase;
	osg::Matrix linkPost;
	for (int i=0; i<vehicle.nlinks;i++) {
	   osg::ref_ptr<osg::Program> program = osgOcean::ShaderManager::instance().createProgram("robot_shader", model_vertex, model_fragment, true);
	   program->addBindAttribLocation("aTangent", 6);

	   link[i]->getOrCreateStateSet()->setAttributeAndModes(program,osg::StateAttribute::ON);
	   link[i]->getStateSet()->addUniform( new osg::Uniform( "uOverlayMap", 1 ) );
	   link[i]->getStateSet()->addUniform( new osg::Uniform( "uNormalMap",  2 ) );

	   link[i]->setNodeMask( oscene->getNormalSceneMask() | oscene->getReflectedSceneMask() | oscene->getRefractedSceneMask() );
	   linkBase.makeIdentity();
	   //linkBase.preMultRotate(osg::Quat(vehicle.links[i].rpy[0],osg::Vec3d(1,0,0)));
	   //linkBase.preMultRotate(osg::Quat(vehicle.links[i].rpy[1],osg::Vec3d(0,1,0)));
	   //linkBase.preMultRotate(osg::Quat(vehicle.links[i].rpy[2],osg::Vec3d(0,0,1)));
	   //linkBase.preMultTranslate(osg::Vec3d(-vehicle.links[i].position[0],-vehicle.links[i].position[1],-vehicle.links[i].position[2]));
	   linkBase.makeTranslate(osg::Vec3d (vehicle.links[i].position[0],vehicle.links[i].position[1],vehicle.links[i].position[2]));
	   linkBase.preMultRotate(osg::Quat (vehicle.links[i].quat[0],vehicle.links[i].quat[1],vehicle.links[i].quat[2],vehicle.links[i].quat[3]));

	   
	   linkBaseTransforms[i]= new osg::MatrixTransform;
	   linkBaseTransforms[i]->setMatrix(linkBase);
	   linkBaseTransforms[i]->addChild(link[i]);

	   //linkBase.invert(linkPost);
	   linkPost.makeRotate(osg::Quat (vehicle.links[i].quat[0],vehicle.links[i].quat[1],vehicle.links[i].quat[2],vehicle.links[i].quat[3]).inverse());
	   linkPost.preMultTranslate(-osg::Vec3d (vehicle.links[i].position[0],vehicle.links[i].position[1],vehicle.links[i].position[2]));
	   linkPostTransforms[i]= new osg::MatrixTransform;
	   linkPostTransforms[i]->setMatrix(linkPost);
	   link[i]->asGroup()->addChild(linkPostTransforms[i]);
	}

	joints.resize(vehicle.njoints);
	zerojoints.resize(vehicle.njoints);

	for (int i=0; i<vehicle.njoints; i++) {
	   joints[i]= new osg::MatrixTransform;
	   zerojoints[i]= new osg::MatrixTransform;
	}

	osg::Matrix m;
	for(int i=0; i<vehicle.njoints; i++){
	   m.makeTranslate(0,0,0);
	   m.preMultTranslate(osg::Vec3d (vehicle.joints[i].position[0],vehicle.joints[i].position[1],vehicle.joints[i].position[2]));
	   m.preMultRotate(osg::Quat (vehicle.joints[i].quat[0],vehicle.joints[i].quat[1],vehicle.joints[i].quat[2],vehicle.joints[i].quat[3]));

	   joints[i]->setMatrix(m);
	   zerojoints[i]->setMatrix(m);
	}

	baseTransform=new osg::MatrixTransform();
	baseTransform->addChild(linkBaseTransforms[0]);
	baseTransform->addChild(axis);
	for (int i=0; i<vehicle.njoints; i++) {
	   linkPostTransforms[vehicle.joints[i].parent]->asGroup()->addChild(joints[i]);
	   joints[i]->addChild(linkBaseTransforms[vehicle.joints[i].child]);
	   joints[i]->addChild(axis);
	}

	//Save rotations for joints update, limits, and type of joints
	joint_axis.resize(vehicle.njoints);
	for(int i=0; i<vehicle.njoints;i++){
	   joint_axis[i]=osg::Vec3d(vehicle.joints[i].axis[0],vehicle.joints[i].axis[1],vehicle.joints[i].axis[2]);
	   jointType[i]=vehicle.joints[i].type;
	   limits[i]=std::pair<double,double>(vehicle.joints[i].lowLimit,vehicle.joints[i].upLimit);
	}
	
	//Save mimic info
	for(int i=0;i<vehicle.njoints;i++){
	   if(vehicle.joints[i].mimicp==-1){
             mimic[i].joint=i;
	     mimic[i].offset=0;
	     mimic[i].multiplier=1;
	     mimic[i].sliderCrank=0;
	   }
	   else{
	     mimic[i].joint=vehicle.joints[i].mimicp;
  	     mimic[i].offset=vehicle.joints[i].mimic->offset;
  	     mimic[i].multiplier=vehicle.joints[i].mimic->multiplier;
	     if(vehicle.joints[i].name.find("slidercrank")!=vehicle.joints[i].name.length())
		mimic[i].sliderCrank=1;
	   }
	}


	osg::notify(osg::ALWAYS) << "Robot successfully loaded. Total time: ";
   }	
}

void URDFRobot::updateJoints(std::vector<double> &q) {
	osg::Matrix m;

	for (int i=0; i<getNumberOfJoints(); i++) {
	  if(jointType[i]==1){
	    if(mimic[i].sliderCrank==0)
	      m.makeRotate(q[mimic[i].joint]*mimic[i].multiplier+mimic[i].offset,joint_axis[i]);   
	    else
	      m.makeRotate((q[mimic[i].joint]+asin(mimic[i].offset*sin(q[mimic[i].joint])))*-1,joint_axis[i]);
	  }
	  else if(jointType[i]==2)
	    m.makeTranslate( joint_axis[i] *= (q[mimic[i].joint]*mimic[i].multiplier+mimic[i].offset) );
	  else
	    m.makeIdentity();
	  osg::Matrix nm=zerojoints[i]->getMatrix();
	  nm.preMult(m);
	  joints[i]->setMatrix(nm);
	}
}


void URDFRobot::updateJoints(std::vector<double> &q, int startJoint, int numJoints) {
	osg::Matrix m;

	for (int i=startJoint; i<startJoint+numJoints; i++) {
		m.makeRotate(q[mimic[i].joint]*mimic[i].multiplier+mimic[i].offset,joint_axis[i]);   
		osg::Matrix nm=zerojoints[i]->getMatrix();
		nm.preMult(m);
		joints[i]->setMatrix(nm);
	}
}


URDFRobot::~URDFRobot() {
}
