
#include "ConfigXMLParser.h"
#include "SimulatorConfig.h"


  void ConfigFile::esPi(string in, double * param){
    in.erase(0, in.find_first_not_of("\t "));
    in.erase(in.find_last_not_of("\t ")+1,-1);  

    if(in=="M_PI")
      *param=M_PI;
    else if (in=="M_PI_2")
      *param=M_PI_2;
    else if (in=="M_PI_4")
      *param=M_PI_4;
    else if(in=="-M_PI")
      *param=-M_PI;
    else if (in=="-M_PI_2")
      *param=-M_PI_2;
    else if (in=="-M_PI_4")
      *param=-M_PI_4;
    else
      *param= atof(in.c_str());	
  }

  void ConfigFile::extractFloatChar(const xmlpp::Node* node,double * param){
    xmlpp::Node::NodeList list = node->get_children();
  
    for(xmlpp::Node::NodeList::iterator iter = list.begin(); iter != list.end(); ++iter){
      const xmlpp::TextNode* nodeText = dynamic_cast<const xmlpp::TextNode*>(*iter);
      if(nodeText)
	esPi(nodeText->get_content(),param);
        //*param=atof(nodeText->get_content().c_str());
    }
  }

  void ConfigFile::extractIntChar(const xmlpp::Node* node,int * param){
    xmlpp::Node::NodeList list = node->get_children();
  
    for(xmlpp::Node::NodeList::iterator iter = list.begin(); iter != list.end(); ++iter){
      const xmlpp::TextNode* nodeText = dynamic_cast<const xmlpp::TextNode*>(*iter);
      if(nodeText)
        *param=atoi(nodeText->get_content().c_str());
    }
  }

  void ConfigFile::extractUIntChar(const xmlpp::Node* node, unsigned int * param){
    xmlpp::Node::NodeList list = node->get_children();
  
    for(xmlpp::Node::NodeList::iterator iter = list.begin(); iter != list.end(); ++iter){
      const xmlpp::TextNode* nodeText = dynamic_cast<const xmlpp::TextNode*>(*iter);
      if(nodeText)
        *param=(unsigned int)atoi(nodeText->get_content().c_str());
    }
  }

  void ConfigFile::extractStringChar(const xmlpp::Node* node,string * param){
    xmlpp::Node::NodeList list = node->get_children();
  
    for(xmlpp::Node::NodeList::iterator iter = list.begin(); iter != list.end(); ++iter){
      const xmlpp::TextNode* nodeText = dynamic_cast<const xmlpp::TextNode*>(*iter);
      if(nodeText)
        *param=nodeText->get_content();
        param->erase(0, param->find_first_not_of("\t "));
	param->erase(param->find_last_not_of("\t ")+1,-1);
    }
  }

  void ConfigFile::extractPositionOrColor(const xmlpp::Node* node,double * param){
    xmlpp::Node::NodeList list = node->get_children();
    for(xmlpp::Node::NodeList::iterator iter = list.begin(); iter != list.end(); ++iter){
      xmlpp::Node* child=dynamic_cast<const xmlpp::Node*>(*iter);

      if(child->get_name()=="x" || child->get_name()=="r")
	extractFloatChar(child,&param[0]);
      else if(child->get_name()=="y" || child->get_name()=="g")
	extractFloatChar(child,&param[1]);
      else if(child->get_name()=="z" || child->get_name()=="b")
	extractFloatChar(child,&param[2]);
    }
  }

  void ConfigFile::extractOrientation(const xmlpp::Node* node,double * param){
    xmlpp::Node::NodeList list = node->get_children();
    for(xmlpp::Node::NodeList::iterator iter = list.begin(); iter != list.end(); ++iter){
      xmlpp::Node* child=dynamic_cast<const xmlpp::Node*>(*iter);

      if(child->get_name()=="r")
	extractFloatChar(child,&param[0]);
      else if(child->get_name()=="p")
	extractFloatChar(child,&param[1]);
      else if(child->get_name()=="y")
	extractFloatChar(child,&param[2]);
    }
  }

 void ConfigFile::processFog(const xmlpp::Node* node){

    xmlpp::Node::NodeList list = node->get_children();
      for(xmlpp::Node::NodeList::iterator iter = list.begin(); iter != list.end(); ++iter){
        xmlpp::Node* child=dynamic_cast<const xmlpp::Node*>(*iter);
	if(child->get_name()=="density")
	  extractFloatChar(child,&fogDensity);
	else if(child->get_name()=="color")
	  extractPositionOrColor(child,fogColor);
      }
  }

  void ConfigFile::processOceanState(const xmlpp::Node* node){

    xmlpp::Node::NodeList list = node->get_children();
      for(xmlpp::Node::NodeList::iterator iter = list.begin(); iter != list.end(); ++iter){
        xmlpp::Node* child=dynamic_cast<const xmlpp::Node*>(*iter);
        //cout<<child->get_name()<<endl;
        if(child->get_name()=="windx")
	  extractFloatChar(child,&windx);
	else if(child->get_name()=="windy")
	  extractFloatChar(child,&windy);
	else if(child->get_name()=="windSpeed")
	  extractFloatChar(child,&windSpeed);
	else if(child->get_name()=="depth")
	  extractFloatChar(child,&depth);
       	else if(child->get_name()=="reflectionDamping")
	  extractFloatChar(child,&reflectionDamping);
	else if(child->get_name()=="waveScale")
	  extractFloatChar(child,&waveScale);
	else if(child->get_name()=="isNotChoppy"){
	  extractIntChar(child,&isNotChoppy);
	  if(isNotChoppy != 0 && isNotChoppy!=1){
	    cout<<"CFG: isNotChoppy is not a binary value ( 0 1), using default value (1)\n";
	    isNotChoppy=1;
	  }
	}
	else if(child->get_name()=="choppyFactor")
	  extractFloatChar(child,&choppyFactor);
	else if(child->get_name()=="crestFoamHeight")
	  extractFloatChar(child,&crestFoamHeight);
	else if(child->get_name()=="oceanSurfaceHeight")
	  extractFloatChar(child,&oceanSurfaceHeight);
	else if(child->get_name()=="fog")
	  processFog(child);
	else if(child->get_name()=="color")
	  extractPositionOrColor(child,color);
	else if(child->get_name()=="attenuation")
	  extractPositionOrColor(child,attenuation);
      }

  }

  void ConfigFile::processSimParams(const xmlpp::Node* node){
    xmlpp::Node::NodeList list = node->get_children();
    for(xmlpp::Node::NodeList::iterator iter = list.begin(); iter != list.end(); ++iter){
      xmlpp::Node* child=dynamic_cast<const xmlpp::Node*>(*iter);


      if(child->get_name()=="disableShaders"){
	extractIntChar(child,&disableShaders);
	if(disableShaders != 0 && disableShaders!=1){
	  cout<<"CFG: disableShaders is not a binary value ( 0 1), using default value (0)\n";
	  disableShaders=0;
	}
      }
      else if(child->get_name()=="eye_in_hand"){
	extractIntChar(child,&eye_in_hand);
	if(eye_in_hand != 0 && eye_in_hand!=1){
	  cout<<"CFG: eye_in_hand is not a binary value ( 0 1), using default value (0)\n";
	  eye_in_hand=0;
	}
      }
      else if(child->get_name()=="resw")
	extractIntChar(child,&resw);
      else if(child->get_name()=="resh")
	extractIntChar(child,&resh);
      else if(child->get_name()=="offsetp")
	extractPositionOrColor(child,offsetp);
      else if(child->get_name()=="offsetr")
	extractPositionOrColor(child,offsetr);

    }
  }

  void ConfigFile::processParameters(const xmlpp::Node* node,Parameters * params){
    xmlpp::Node::NodeList list = node->get_children();
    for(xmlpp::Node::NodeList::iterator iter = list.begin(); iter != list.end(); ++iter){
      xmlpp::Node* child=dynamic_cast<const xmlpp::Node*>(*iter);
 	
      if(child->get_name()=="fx")
	extractFloatChar(child,&params->fx);
      else if(child->get_name()=="fy")
	extractFloatChar(child,&params->fy);
      else if(child->get_name()=="x0")
	extractFloatChar(child,&params->x0);
      else if(child->get_name()=="y0")
	extractFloatChar(child,&params->y0);
      else if(child->get_name()=="f")
	extractFloatChar(child,&params->f);
      else if(child->get_name()=="n")
	extractFloatChar(child,&params->n);
      else if(child->get_name()=="k")
	extractFloatChar(child,&params->k);

    }
  }

  void ConfigFile::processVcam(const xmlpp::Node* node,Vcam * vcam){
    xmlpp::Node::NodeList list = node->get_children();
    for(xmlpp::Node::NodeList::iterator iter = list.begin(); iter != list.end(); ++iter){
      xmlpp::Node* child=dynamic_cast<const xmlpp::Node*>(*iter);

      if(child->get_name()=="resw")
	extractIntChar(child,&vcam->resw);
      else if(child->get_name()=="resh")
	extractIntChar(child,&vcam->resh);
      else if(child->get_name()=="position")
	extractPositionOrColor(child,vcam->position);
      else if(child->get_name()=="relativeTo")
	extractStringChar(child,&vcam->linkName);
      else if(child->get_name()=="orientation")
	extractOrientation(child,vcam->orientation);
      else if(child->get_name()=="name")
	extractStringChar(child,&vcam->name);
      else if(child->get_name()=="parameters"){
	vcam->parameters=new Parameters();
	processParameters(child,vcam->parameters);

      }
    }

  }

  void ConfigFile::processRangeSensor(const xmlpp::Node* node, rangeSensor *rs){
    xmlpp::Node::NodeList list = node->get_children();
    for(xmlpp::Node::NodeList::iterator iter = list.begin(); iter != list.end(); ++iter){
      xmlpp::Node* child=dynamic_cast<const xmlpp::Node*>(*iter);

      if(child->get_name()=="position")
	extractPositionOrColor(child,rs->position);
      else if(child->get_name()=="relativeTo")
	extractStringChar(child,&rs->linkName);
      else if(child->get_name()=="orientation")
	extractOrientation(child,rs->orientation);
      else if(child->get_name()=="name")
	extractStringChar(child,&rs->name);
      else if(child->get_name()=="range")
	extractFloatChar(child,&rs->range);
      else if(child->get_name()=="visible")
	extractIntChar(child,&rs->visible);
    }
  }

  void ConfigFile::processCamera(const xmlpp::Node* node){
    xmlpp::Node::NodeList list = node->get_children();
    for(xmlpp::Node::NodeList::iterator iter = list.begin(); iter != list.end(); ++iter){
      xmlpp::Node* child=dynamic_cast<const xmlpp::Node*>(*iter);

      if(child->get_name()=="freeMotion"){
	extractIntChar(child,&freeMotion);
	if (freeMotion!=0 && freeMotion!=1) {
	  cout<<"CFG: freeMotion is not a binary value ( 0 1), using default value (1)\n";
	  freeMotion=1;
	}
      }
      else if(child->get_name()=="fov")
	extractFloatChar(child,&camFov);
      else if(child->get_name()=="aspectRatio")
	extractFloatChar(child,&camAspectRatio);
      else if(child->get_name()=="near")
	extractFloatChar(child,&camNear);
      else if(child->get_name()=="far")
	extractFloatChar(child,&camFar);
      else if(child->get_name()=="position")
	extractPositionOrColor(child,camPosition);
      else if(child->get_name()=="lookAt")
	extractPositionOrColor(child,camLookAt);
      if(child->get_name()=="objectToTrack")
	extractStringChar(child,&vehicleToTrack);
    }
  }

  void ConfigFile::processPose(urdf::Pose pose,double * position, double * rpy,double * quat){
    position[0]=pose.position.x;
    position[1]=pose.position.y;
    position[2]=pose.position.z;
    pose.rotation.getRPY(rpy[0],rpy[1],rpy[2]);
    pose.rotation.getQuaternion(quat[0],quat[1],quat[2],quat[3]);
  }

  int ConfigFile::processVisual(boost::shared_ptr<const urdf::Visual> visual,Link * link,int nmat,Material * materials){
     urdf::Geometry * geometry =visual->geometry.get();


     if (geometry->type ==urdf::Geometry::MESH){
        urdf::Mesh *mesh = dynamic_cast<urdf::Mesh*>(geometry);
        link->file=mesh->filename;
	link->type=0;
     }
     else if(geometry->type ==urdf::Geometry::BOX){
        urdf::Box *box = dynamic_cast<urdf::Box*>(geometry);
	link->type=1;
	link->boxSize[0]=box->dim.x;
	link->boxSize[1]=box->dim.y;
	link->boxSize[2]=box->dim.z;
     }
     else if(geometry->type ==urdf::Geometry::CYLINDER){
        urdf::Cylinder *cylinder = dynamic_cast<urdf::Cylinder*>(geometry);
	link->type=2;
	link->length=cylinder->length;
	link->radius=cylinder->radius;
     }
     else if(geometry->type ==urdf::Geometry::SPHERE){
        urdf::Sphere *sphere = dynamic_cast<urdf::Sphere*>(geometry);
	link->type=3;
	link->radius=sphere->radius;
     }
     processPose(visual->origin,link->position,link->rpy,link->quat);


     //Material
	//Search if it's a new materia
     int found=0;
     if(visual->material!=NULL){
       for(int i=0;i<nmat && !found;i++){
	 if(visual->material_name==materials[i].name){
	   link->material=i;
	   found=1;
	 }
       }

       if(!found){
	 materials[nmat].name=visual->material_name;
	 materials[nmat].r=visual->material->color.r;
	 materials[nmat].g=visual->material->color.g;
	 materials[nmat].b=visual->material->color.b;
	 materials[nmat].a=visual->material->color.a;
	 link->material=nmat;
	 nmat++;
       }
     }
     else
	link->material=-1;
     return nmat;
  }

  void ConfigFile::processJoint(boost::shared_ptr<const urdf::Joint> joint,Joint * jointVehicle,int parentLink,int childLink){

    jointVehicle->name=joint->name;
    jointVehicle->axis[0]=joint->axis.x;
    jointVehicle->axis[1]=joint->axis.y;
    jointVehicle->axis[2]=joint->axis.z;
    processPose(joint->parent_to_joint_origin_transform,jointVehicle->position,jointVehicle->rpy,jointVehicle->quat);
    jointVehicle->child=childLink;
    jointVehicle->parent=parentLink;

    if(joint->type==6)
	jointVehicle->type=0;
    else if(joint->type==1 || joint->type==2)
	jointVehicle->type=1;
    else if(joint->type==3)
	jointVehicle->type=2;
    else{
	cout<<"Unsupported type of joint in "<<joint->name<<", fixed joint will be used."<<endl;
	jointVehicle->type=0;
    }

    //Mimic
    if(joint->mimic!=NULL){
      jointVehicle->mimic= new Mimic;
      jointVehicle->mimic->jointName=joint->mimic->joint_name;
      jointVehicle->mimic->offset=joint->mimic->offset;
      jointVehicle->mimic->multiplier=joint->mimic->multiplier;
    }
    else
      jointVehicle->mimic=NULL;

    //limits
    if(joint->limits!=NULL){
      jointVehicle->lowLimit=joint->limits->lower;
      jointVehicle->upLimit=joint->limits->upper;
    }
    else{
      jointVehicle->lowLimit=-M_PI;
      jointVehicle->upLimit=M_PI;
    }
  }

  int ConfigFile::processLink(boost::shared_ptr<const urdf::Link> link,Vehicle * vehicle,int nlink,int njoint,int nmat,Material * materials){

    vehicle->links[nlink].name=link->name;

    nmat= processVisual(link->visual,&vehicle->links[nlink],nmat, materials);

    int linkNumber=nlink;
    for(uint i=0;i<link->child_joints.size();i++){
       processJoint(link->child_joints[i],&vehicle->joints[linkNumber],nlink,linkNumber+1);
       linkNumber=processLink(link->child_links[i],vehicle,linkNumber+1,linkNumber+1,nmat,materials);
    }
    return linkNumber;
  }

  int ConfigFile::processURDFFile(string file, Vehicle * vehicle){
    urdf::Model model;

    if (!model.initFile(std::string(SIMULATOR_ROOT_PATH)+file)){	
      ROS_ERROR("Failed to parse urdf file");
      return -1;
    }

    ROS_INFO("Successfully parsed urdf file");

    vehicle->nlinks=model.links_.size();
    vehicle->links = new Link[vehicle->nlinks];
    vehicle->njoints=model.joints_.size();
    vehicle->joints = new Joint[vehicle->njoints];
    vehicle->nmaterials = model.materials_.size();
    vehicle->materials = new Material[vehicle->nmaterials];
    boost::shared_ptr<const urdf::Link> root = model.getRoot();
    processLink(root,vehicle,0,0,0,vehicle->materials);
    return 0;
  }

  void ConfigFile::processJointValues(const xmlpp::Node* node,double ** jointValues,int * ninitJoints){
    xmlpp::Node::NodeList list = node->get_children();
    *ninitJoints=(list.size()-1)/2;
    *jointValues = new double[(list.size()-1)/2];
    int pos=0;
    for(xmlpp::Node::NodeList::iterator iter = list.begin(); iter != list.end(); ++iter){
      xmlpp::Node* child=dynamic_cast<const xmlpp::Node*>(*iter);
      if(child->get_name()=="joint"){
        extractFloatChar(child,&(*jointValues)[pos]);
	pos++;
      }
    }
  }

  void ConfigFile::postprocessVehicle(Vehicle * vehicle){

    //Get mimic parents

    int found;
    for(int i=0;i<vehicle->njoints;i++){
      if(vehicle->joints[i].mimic!=NULL){
        found=0;
	for(int j=0;j<vehicle->njoints && !found;j++){
	  if(vehicle->joints[j].name==vehicle->joints[i].mimic->jointName){
	    vehicle->joints[i].mimicp=j;
            found=1;
          }
	}
        if(found==0){
	  vehicle->joints[i].mimicp=-1;
        }
      }
      else{
	vehicle->joints[i].mimicp=-1;
	}
    }

    //get camera joint
    Vcam aux;
    for(unsigned int i=0;i<vehicle->Vcams.size();i++){
      found=0;
      aux=vehicle->Vcams.front();
      vehicle->Vcams.pop_front();
      for(int j=0;j<vehicle->nlinks && !found;j++){
	if(vehicle->links[j].name==aux.linkName){
	  aux.link=j;
	  found=1;
	}
      }
      if(found==0)
	cout<<"Camera attached to unknown link: "<<aux.linkName<<" camera will be ignored"<<endl;
      else 
      	vehicle->Vcams.push_back(aux);
    }

    //get Range sensor joint
    rangeSensor rs;
    for(unsigned int i=0;i<vehicle->range_sensors.size();i++){
      found=0;
      rs=vehicle->range_sensors.front();
      vehicle->range_sensors.pop_front();
      for(int j=0;j<vehicle->nlinks && !found;j++){
	if(vehicle->links[j].name==rs.linkName){
	  rs.link=j;
	  found=1;
	}
      }
      if(found==0)
	cout<<"RangeSensor attached to unknown link: "<<rs.linkName<<". Will be ignored"<<endl;
      else 
      	vehicle->range_sensors.push_back(rs);
    }
  }

  void ConfigFile::processVehicle(const xmlpp::Node* node,Vehicle *vehicle){
    xmlpp::Node::NodeList list = node->get_children();
    for(xmlpp::Node::NodeList::iterator iter = list.begin(); iter != list.end(); ++iter){
      xmlpp::Node* child=dynamic_cast<const xmlpp::Node*>(*iter);

      if(child->get_name()=="name")
	extractStringChar(child,&vehicle->name);
      else if(child->get_name()=="file"){
	string aux;
	extractStringChar(child,&aux);
	processURDFFile(aux,vehicle);
      }
      else if(child->get_name()=="position")
	extractPositionOrColor(child,vehicle->position);
      else if(child->get_name()=="orientation")
	extractOrientation(child,vehicle->orientation);
      else if (child->get_name()=="jointValues")
	processJointValues(child,&vehicle->jointValues,&vehicle->ninitJoints);
      else if (child->get_name()=="virtualCamera"){
	Vcam aux;
	aux.init();
	processVcam(child,&aux);
	vehicle->Vcams.push_back(aux);	
      } else if (child->get_name()=="rangeSensor"){
	rangeSensor aux;
	aux.init();
	processRangeSensor(child,&aux);
	vehicle->range_sensors.push_back(aux);	
      }
    }
  }

  

  void ConfigFile::processObject(const xmlpp::Node* node,Object *object){
    xmlpp::Node::NodeList list = node->get_children();
    for(xmlpp::Node::NodeList::iterator iter = list.begin(); iter != list.end(); ++iter){
      xmlpp::Node* child=dynamic_cast<const xmlpp::Node*>(*iter);

      if(child->get_name()=="name")
	extractStringChar(child,&object->name);
      else if(child->get_name()=="file")
	extractStringChar(child,&object->file);
      else if(child->get_name()=="position")
	extractPositionOrColor(child,object->position);
      else if(child->get_name()=="orientation")
	extractOrientation(child,object->orientation);
      else if(child->get_name()=="offsetp")
	extractPositionOrColor(child,object->offsetp);
      else if(child->get_name()=="offsetr")
	extractPositionOrColor(child,object->offsetr);

    }
  }

  void ConfigFile::processROSInterface(const xmlpp::Node* node,ROSInterfaceInfo * rosInterface){
    xmlpp::Node::NodeList list = node->get_children();
    for(xmlpp::Node::NodeList::iterator iter = list.begin(); iter != list.end(); ++iter){
      xmlpp::Node* child=dynamic_cast<const xmlpp::Node*>(*iter);

      if(child->get_name()=="topic" || child->get_name()=="imageTopic" )
	extractStringChar(child,&rosInterface->topic);
      else if(child->get_name()=="vehicleName" || child->get_name()=="cameraName" || child->get_name()=="name")
	extractStringChar(child,&rosInterface->targetName);
      else if(child->get_name()=="rate")
	extractIntChar(child,&rosInterface->rate);
      else if(child->get_name()=="infoTopic")
	extractStringChar(child,&rosInterface->infoTopic);
      else if(child->get_name()=="width")
	extractUIntChar(child,&rosInterface->w);
      else if(child->get_name()=="height")
	extractUIntChar(child,&rosInterface->h);
      else if(child->get_name()=="posx")
	extractUIntChar(child,&rosInterface->posx);
      else if(child->get_name()=="posy")
	extractUIntChar(child,&rosInterface->posy);
      else if(child->get_name()=="scale")
	extractFloatChar(child,&rosInterface->scale);
    }
  }

  void ConfigFile::processROSInterfaces(const xmlpp::Node* node){
    xmlpp::Node::NodeList list = node->get_children();
    for(xmlpp::Node::NodeList::iterator iter = list.begin(); iter != list.end(); ++iter){
      xmlpp::Node* child=dynamic_cast<const xmlpp::Node*>(*iter);

      ROSInterfaceInfo rosInterface;
      rosInterface.posx=rosInterface.posy=0;
      rosInterface.scale=1;
      if(child->get_name()=="ROSOdomToPAT"){
	rosInterface.type=ROSInterfaceInfo::ROSOdomToPAT;
	rosInterface.rate=10;	//TODO: Set rate on XML
	processROSInterface(child,&rosInterface);
	ROSInterfaces.push_back(rosInterface);
      }
      else if(child->get_name()=="PATToROSOdom"){
	rosInterface.type=ROSInterfaceInfo::PATToROSOdom;
	rosInterface.rate=10;
	processROSInterface(child,&rosInterface);
	ROSInterfaces.push_back(rosInterface);
      }
      else if(child->get_name()=="ArmToROSJointState") {
	rosInterface.type=ROSInterfaceInfo::ArmToROSJointState;
	rosInterface.rate=10;
	processROSInterface(child,&rosInterface);
	ROSInterfaces.push_back(rosInterface);
      } else if(child->get_name()=="ROSJointStateToArm") {
	rosInterface.type=ROSInterfaceInfo::ROSJointStateToArm;
	rosInterface.rate=10;
	processROSInterface(child,&rosInterface);
	ROSInterfaces.push_back(rosInterface);
      } else if(child->get_name()=="VirtualCameraToROSImage") {
	rosInterface.type=ROSInterfaceInfo::VirtualCameraToROSImage;
	rosInterface.rate=25;
	processROSInterface(child,&rosInterface);
	ROSInterfaces.push_back(rosInterface);	
      } else if(child->get_name()=="RangeSensorToROSRange") {
	rosInterface.type=ROSInterfaceInfo::RangeSensorToROSRange;
	rosInterface.rate=10;
	processROSInterface(child,&rosInterface);
	ROSInterfaces.push_back(rosInterface);	
      } else if(child->get_name()=="ROSImageToHUD") {
	rosInterface.type=ROSInterfaceInfo::ROSImageToHUD;
	processROSInterface(child,&rosInterface);
	ROSInterfaces.push_back(rosInterface);	
      } 
	else if(child->get_name()=="ROSTwistToPAT"){
	rosInterface.type=ROSInterfaceInfo::ROSTwistToPAT;
	rosInterface.rate=10;
	processROSInterface(child,&rosInterface);
	ROSInterfaces.push_back(rosInterface);
      }

    }
  }

  void ConfigFile::processXML(const xmlpp::Node* node){

    if(node->get_name()!="UWSimScene")
      cout<<"CFG: XML file is not UWSimScene file."<<endl;
    else{

      xmlpp::Node::NodeList list = node->get_children();
      for(xmlpp::Node::NodeList::iterator iter = list.begin(); iter != list.end(); ++iter)
      {
        xmlpp::Node* child=dynamic_cast<const xmlpp::Node*>(*iter);
        //cout<<child->get_name()<<endl;
        if(child->get_name()=="oceanState")
  	  processOceanState(child);
        else if(child->get_name()=="simParams")
	  processSimParams(child);
	else if(child->get_name()=="camera")
	  processCamera(child);
	else if(child->get_name()=="vehicle"){
	  Vehicle  vehicle;
	  vehicle.jointValues=NULL;
	  processVehicle(child,&vehicle);
          postprocessVehicle(&vehicle);
	  vehicles.push_back(vehicle);		
	}
	else if(child->get_name()=="object"){
	  Object  object;
	  memset(object.offsetp,0,3*sizeof(double));
 	  memset(object.offsetr,0,3*sizeof(double));
	  processObject(child,&object);
	  objects.push_back(object);		
	}
        else if(child->get_name()=="rosInterfaces")
	  processROSInterfaces(child);
      }
    }

  }


  ConfigFile::ConfigFile(const std::string &fName){
    memset(offsetr,0,3*sizeof(double));
    memset(offsetp,0,3*sizeof(double));
    camNear=camFar=-1;

    try
    {
      xmlpp::DomParser parser;
      parser.set_validate();
      parser.set_substitute_entities(); //We just want the text to be resolved/unescaped automatically.
      parser.parse_file(fName);
      if(parser)
      {
        //Walk the tree:
        const xmlpp::Node* pNode = parser.get_document()->get_root_node(); //deleted by DomParser.
        processXML(pNode);
      }
    }
    catch(const std::exception& ex)
    {
      std::cerr << "Exception caught: " << ex.what() << std::endl;
      std::cerr << "Please check your XML file" << std::endl;
      exit(0);
    }

  }






