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

#include "ConfigXMLParser.h"
#include "SimulatorConfig.h"
#include <osgDB/FileUtils>

void ConfigFile::esPi(string in, double &param){
	in.erase(0, in.find_first_not_of("\t "));
	in.erase(in.find_last_not_of("\t ")+1,-1);

	if(in=="M_PI")
		param=M_PI;
	else if (in=="M_PI_2")
		param=M_PI_2;
	else if (in=="M_PI_4")
		param=M_PI_4;
	else if(in=="-M_PI")
		param=-M_PI;
	else if (in=="-M_PI_2")
		param=-M_PI_2;
	else if (in=="-M_PI_4")
		param=-M_PI_4;
	else
		param= boost::lexical_cast<double>(in.c_str());
}

void ConfigFile::extractFloatChar(const xmlpp::Node* node,double &param){
	xmlpp::Node::NodeList list = node->get_children();

	for(xmlpp::Node::NodeList::iterator iter = list.begin(); iter != list.end(); ++iter){
		const xmlpp::TextNode* nodeText = dynamic_cast<const xmlpp::TextNode*>(*iter);
		if(nodeText)
			esPi(nodeText->get_content(),param);
		//*param=atof(nodeText->get_content().c_str());
	}
}

void ConfigFile::extractIntChar(const xmlpp::Node* node,int &param){
	xmlpp::Node::NodeList list = node->get_children();

	for(xmlpp::Node::NodeList::iterator iter = list.begin(); iter != list.end(); ++iter){
		const xmlpp::TextNode* nodeText = dynamic_cast<const xmlpp::TextNode*>(*iter);
		if(nodeText)
			param=atoi(nodeText->get_content().c_str());
	}
}

void ConfigFile::extractUIntChar(const xmlpp::Node* node, unsigned int &param){
	xmlpp::Node::NodeList list = node->get_children();

	for(xmlpp::Node::NodeList::iterator iter = list.begin(); iter != list.end(); ++iter){
		const xmlpp::TextNode* nodeText = dynamic_cast<const xmlpp::TextNode*>(*iter);
		if(nodeText)
			param=(unsigned int)atoi(nodeText->get_content().c_str());
	}
}

void ConfigFile::extractStringChar(const xmlpp::Node* node, string &param){
	xmlpp::Node::NodeList list = node->get_children();

	for(xmlpp::Node::NodeList::iterator iter = list.begin(); iter != list.end(); ++iter){
		const xmlpp::TextNode* nodeText = dynamic_cast<const xmlpp::TextNode*>(*iter);
		if(nodeText)
			param=nodeText->get_content();
		param.erase(0, param.find_first_not_of("\t "));
		param.erase(param.find_last_not_of("\t ")+1,-1);
	}
}

void ConfigFile::extractPositionOrColor(const xmlpp::Node* node,double param[3]){
	xmlpp::Node::NodeList list = node->get_children();
	for(xmlpp::Node::NodeList::iterator iter = list.begin(); iter != list.end(); ++iter){
		xmlpp::Node* child=dynamic_cast<const xmlpp::Node*>(*iter);

		if(child->get_name()=="x" || child->get_name()=="r")
			extractFloatChar(child,param[0]);
		else if(child->get_name()=="y" || child->get_name()=="g")
			extractFloatChar(child,param[1]);
		else if(child->get_name()=="z" || child->get_name()=="b")
			extractFloatChar(child,param[2]);
	}
}

void ConfigFile::extractOrientation(const xmlpp::Node* node,double param[3]){
	xmlpp::Node::NodeList list = node->get_children();
	for(xmlpp::Node::NodeList::iterator iter = list.begin(); iter != list.end(); ++iter){
		xmlpp::Node* child=dynamic_cast<const xmlpp::Node*>(*iter);

		if(child->get_name()=="r")
			extractFloatChar(child,param[0]);
		else if(child->get_name()=="p")
			extractFloatChar(child,param[1]);
		else if(child->get_name()=="y")
			extractFloatChar(child,param[2]);
	}
}

void ConfigFile::processFog(const xmlpp::Node* node){

	xmlpp::Node::NodeList list = node->get_children();
	for(xmlpp::Node::NodeList::iterator iter = list.begin(); iter != list.end(); ++iter){
		xmlpp::Node* child=dynamic_cast<const xmlpp::Node*>(*iter);
		if(child->get_name()=="density")
			extractFloatChar(child,fogDensity);
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
			extractFloatChar(child,windx);
		else if(child->get_name()=="windy")
			extractFloatChar(child,windy);
		else if(child->get_name()=="windSpeed")
			extractFloatChar(child,windSpeed);
		else if(child->get_name()=="depth")
			extractFloatChar(child,depth);
		else if(child->get_name()=="reflectionDamping")
			extractFloatChar(child,reflectionDamping);
		else if(child->get_name()=="waveScale")
			extractFloatChar(child,waveScale);
		else if(child->get_name()=="isNotChoppy"){
			extractIntChar(child,isNotChoppy);
			if(isNotChoppy != 0 && isNotChoppy!=1){
				OSG_WARN << "ConfigFile::processOceanState: isNotChoppy is not a binary value ( 0 1), using default value (1)" << std::endl;
				isNotChoppy=1;
			}
		}
		else if(child->get_name()=="choppyFactor")
			extractFloatChar(child,choppyFactor);
		else if(child->get_name()=="crestFoamHeight")
			extractFloatChar(child,crestFoamHeight);
		else if(child->get_name()=="oceanSurfaceHeight")
			extractFloatChar(child,oceanSurfaceHeight);
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
			extractIntChar(child,disableShaders);
			if(disableShaders != 0 && disableShaders!=1){
				OSG_WARN <<"ConfigFile::processSimParams: disableShaders is not a binary value ( 0 1), using default value (0)" << std::endl;
				disableShaders=0;
			}
		}
		else if(child->get_name()=="eye_in_hand"){
			extractIntChar(child,eye_in_hand);
			if(eye_in_hand != 0 && eye_in_hand!=1){
				OSG_WARN <<"ConfigFile::processSimParams: eye_in_hand is not a binary value ( 0 1), using default value (0)" << std::endl;
				eye_in_hand=0;
			}
		}
		else if(child->get_name()=="resw")
			extractIntChar(child,resw);
		else if(child->get_name()=="resh")
			extractIntChar(child,resh);
		else if(child->get_name()=="offsetp")
			extractPositionOrColor(child,offsetp);
		else if(child->get_name()=="offsetr")
			extractPositionOrColor(child,offsetr);
     		else if(child->get_name()=="gravity")
			extractPositionOrColor(child,gravity);
		else if(child->get_name()=="enablePhysics"){
			extractIntChar(child,enablePhysics);
			if(enablePhysics != 0 && enablePhysics!=1){
				OSG_WARN << "ConfigFile::processSimParams: enablePhysics is not a binary value ( 0 1), using default value (0)" << std::endl;
				enablePhysics=0;
			}
			physicsWater.init();
		}
		else if(child->get_name()=="physicsWater"){		
			physicsWater.enable=1;
			processPhysicsWater(child);
		}

	}
}

void ConfigFile::processPhysicsWater(const xmlpp::Node* node){
	xmlpp::Node::NodeList list = node->get_children();
	for(xmlpp::Node::NodeList::iterator iter = list.begin(); iter != list.end(); ++iter){
		xmlpp::Node* child=dynamic_cast<const xmlpp::Node*>(*iter);
		if(child->get_name()=="position")
			extractPositionOrColor(child,physicsWater.position);
		else if(child->get_name()=="size")
			processSize(child);
		else if(child->get_name()=="resolution")
			extractFloatChar(child,physicsWater.resolution);
	}
}

void ConfigFile::processSize(const xmlpp::Node* node){
	xmlpp::Node::NodeList list = node->get_children();
	for(xmlpp::Node::NodeList::iterator iter = list.begin(); iter != list.end(); ++iter){
		xmlpp::Node* child=dynamic_cast<const xmlpp::Node*>(*iter);
		if(child->get_name()=="minX")
			extractFloatChar(child,physicsWater.size[0]);
		else if(child->get_name()=="maxX")
			extractFloatChar(child,physicsWater.size[1]);
		else if(child->get_name()=="minY")
			extractFloatChar(child,physicsWater.size[2]);
		else if(child->get_name()=="maxY")
			extractFloatChar(child,physicsWater.size[3]);
		else if(child->get_name()=="minZ")
			extractFloatChar(child,physicsWater.size[4]);
		else if(child->get_name()=="maxZ")
			extractFloatChar(child,physicsWater.size[5]);

	}
}

void ConfigFile::processParameters(const xmlpp::Node* node, Parameters *params){
	xmlpp::Node::NodeList list = node->get_children();
	for(xmlpp::Node::NodeList::iterator iter = list.begin(); iter != list.end(); ++iter){
		xmlpp::Node* child=dynamic_cast<const xmlpp::Node*>(*iter);

		if(child->get_name()=="fx")
			extractFloatChar(child,params->fx);
		else if(child->get_name()=="fy")
			extractFloatChar(child,params->fy);
		else if(child->get_name()=="x0")
			extractFloatChar(child,params->x0);
		else if(child->get_name()=="y0")
			extractFloatChar(child,params->y0);
		else if(child->get_name()=="f")
			extractFloatChar(child,params->f);
		else if(child->get_name()=="n")
			extractFloatChar(child,params->n);
		else if(child->get_name()=="k")
			extractFloatChar(child,params->k);

	}
}

void ConfigFile::processVcam(const xmlpp::Node* node,Vcam &vcam){
	xmlpp::Node::NodeList list = node->get_children();
	for(xmlpp::Node::NodeList::iterator iter = list.begin(); iter != list.end(); ++iter){
		xmlpp::Node* child=dynamic_cast<const xmlpp::Node*>(*iter);

		if(child->get_name()=="resw")
			extractIntChar(child,vcam.resw);
		else if(child->get_name()=="resh")
			extractIntChar(child,vcam.resh);
		else if(child->get_name()=="position")
			extractPositionOrColor(child,vcam.position);
		else if(child->get_name()=="relativeTo")
			extractStringChar(child,vcam.linkName);
		else if(child->get_name()=="orientation")
			extractOrientation(child,vcam.orientation);
		else if(child->get_name()=="name")
			extractStringChar(child,vcam.name);
		else if(child->get_name()=="baseline"){
			extractFloatChar(child,vcam.baseLine);
		} else if(child->get_name()=="frameId"){
			extractStringChar(child,vcam.frameId);
		} else if(child->get_name()=="parameters"){
			vcam.parameters.reset(new Parameters());
			processParameters(child,vcam.parameters.get());
		} else if(child->get_name()=="showpath")
			extractFloatChar(child,vcam.showpath);
	}

}

void ConfigFile::processRangeSensor(const xmlpp::Node* node, rangeSensor &rs){
	xmlpp::Node::NodeList list = node->get_children();
	for(xmlpp::Node::NodeList::iterator iter = list.begin(); iter != list.end(); ++iter){
		xmlpp::Node* child=dynamic_cast<const xmlpp::Node*>(*iter);

		if(child->get_name()=="position")
			extractPositionOrColor(child,rs.position);
		else if(child->get_name()=="relativeTo")
			extractStringChar(child,rs.linkName);
		else if(child->get_name()=="orientation")
			extractOrientation(child,rs.orientation);
		else if(child->get_name()=="name")
			extractStringChar(child,rs.name);
		else if(child->get_name()=="range")
			extractFloatChar(child,rs.range);
		else if(child->get_name()=="visible")
			extractIntChar(child,rs.visible);
	}
}

void ConfigFile::processImu(const xmlpp::Node* node, Imu &imu){
	xmlpp::Node::NodeList list = node->get_children();
	for(xmlpp::Node::NodeList::iterator iter = list.begin(); iter != list.end(); ++iter){
		xmlpp::Node* child=dynamic_cast<const xmlpp::Node*>(*iter);

		if(child->get_name()=="position")
			extractPositionOrColor(child,imu.position);
		else if(child->get_name()=="relativeTo")
			extractStringChar(child,imu.linkName);
		else if(child->get_name()=="orientation")
			extractOrientation(child,imu.orientation);
		else if(child->get_name()=="name")
			extractStringChar(child,imu.name);
		else if(child->get_name()=="std")
			extractFloatChar(child,imu.std);
	}
}

void ConfigFile::processPressureSensor(const xmlpp::Node* node, XMLPressureSensor &ps){
	xmlpp::Node::NodeList list = node->get_children();
	for(xmlpp::Node::NodeList::iterator iter = list.begin(); iter != list.end(); ++iter){
		xmlpp::Node* child=dynamic_cast<const xmlpp::Node*>(*iter);

		if(child->get_name()=="position")
			extractPositionOrColor(child,ps.position);
		else if(child->get_name()=="relativeTo")
			extractStringChar(child,ps.linkName);
		else if(child->get_name()=="orientation")
			extractOrientation(child,ps.orientation);
		else if(child->get_name()=="name")
			extractStringChar(child,ps.name);
		else if(child->get_name()=="std")
			extractFloatChar(child,ps.std);
	}
}

void ConfigFile::processGPSSensor(const xmlpp::Node* node, XMLGPSSensor &s){
	xmlpp::Node::NodeList list = node->get_children();
	for(xmlpp::Node::NodeList::iterator iter = list.begin(); iter != list.end(); ++iter){
		xmlpp::Node* child=dynamic_cast<const xmlpp::Node*>(*iter);

		if(child->get_name()=="position")
			extractPositionOrColor(child,s.position);
		else if(child->get_name()=="relativeTo")
			extractStringChar(child,s.linkName);
		else if(child->get_name()=="orientation")
			extractOrientation(child,s.orientation);
		else if(child->get_name()=="name")
			extractStringChar(child,s.name);
		else if(child->get_name()=="std")
			extractFloatChar(child,s.std);
	}
}

void ConfigFile::processDVLSensor(const xmlpp::Node* node, XMLDVLSensor &s){
	xmlpp::Node::NodeList list = node->get_children();
	for(xmlpp::Node::NodeList::iterator iter = list.begin(); iter != list.end(); ++iter){
		xmlpp::Node* child=dynamic_cast<const xmlpp::Node*>(*iter);

		if(child->get_name()=="position")
			extractPositionOrColor(child,s.position);
		else if(child->get_name()=="relativeTo")
			extractStringChar(child,s.linkName);
		else if(child->get_name()=="orientation")
			extractOrientation(child,s.orientation);
		else if(child->get_name()=="name")
			extractStringChar(child,s.name);
		else if(child->get_name()=="std")
			extractFloatChar(child,s.std);
	}
}

void ConfigFile::processCamera(const xmlpp::Node* node){
	xmlpp::Node::NodeList list = node->get_children();
	for(xmlpp::Node::NodeList::iterator iter = list.begin(); iter != list.end(); ++iter){
		xmlpp::Node* child=dynamic_cast<const xmlpp::Node*>(*iter);

		if(child->get_name()=="freeMotion"){
			extractIntChar(child,freeMotion);
			if (freeMotion!=0 && freeMotion!=1) {
				OSG_WARN <<"ConfigFile::processCamera: freeMotion is not a binary value ( 0 1), using default value (1)" << std::endl;
				freeMotion=1;
			}
		}
		else if(child->get_name()=="fov")
			extractFloatChar(child,camFov);
		else if(child->get_name()=="aspectRatio")
			extractFloatChar(child,camAspectRatio);
		else if(child->get_name()=="near")
			extractFloatChar(child,camNear);
		else if(child->get_name()=="far")
			extractFloatChar(child,camFar);
		else if(child->get_name()=="position")
			extractPositionOrColor(child,camPosition);
		else if(child->get_name()=="lookAt")
			extractPositionOrColor(child,camLookAt);
		if(child->get_name()=="objectToTrack")
			extractStringChar(child,vehicleToTrack);
	}
}

void ConfigFile::processMultibeamSensor(const xmlpp::Node* node, XMLMultibeamSensor &MB){
	xmlpp::Node::NodeList list = node->get_children();
	for(xmlpp::Node::NodeList::iterator iter = list.begin(); iter != list.end(); ++iter){
		xmlpp::Node* child=dynamic_cast<const xmlpp::Node*>(*iter);

		if(child->get_name()=="position")
			extractPositionOrColor(child,MB.position);
		else if(child->get_name()=="relativeTo")
			extractStringChar(child,MB.linkName);
		else if(child->get_name()=="orientation")
			extractOrientation(child,MB.orientation);
		else if(child->get_name()=="name")
			extractStringChar(child,MB.name);
		else if(child->get_name()=="initAngle")
			extractFloatChar(child,MB.initAngle);
		else if(child->get_name()=="finalAngle")
			extractFloatChar(child,MB.finalAngle);
		else if(child->get_name()=="angleIncr")
			extractFloatChar(child,MB.angleIncr);
		else if(child->get_name()=="range")
			extractFloatChar(child,MB.range);
	}
}

void ConfigFile::processPose(urdf::Pose pose, double position[3], double rpy[3], double quat[4]){
	position[0]=pose.position.x;
	position[1]=pose.position.y;
	position[2]=pose.position.z;
	pose.rotation.getRPY(rpy[0],rpy[1],rpy[2]);
	pose.rotation.getQuaternion(quat[0],quat[1],quat[2],quat[3]);
}

void ConfigFile::processGeometry(urdf::Geometry * geometry, Geometry * geom){

     if (geometry->type ==urdf::Geometry::MESH){
        urdf::Mesh *mesh = dynamic_cast<urdf::Mesh*>(geometry);
        geom->file=mesh->filename;
	geom->type=0;
     }
     else if(geometry->type ==urdf::Geometry::BOX){
        urdf::Box *box = dynamic_cast<urdf::Box*>(geometry);
	geom->type=1;
	geom->boxSize[0]=box->dim.x;
	geom->boxSize[1]=box->dim.y;
	geom->boxSize[2]=box->dim.z;
     }
     else if(geometry->type ==urdf::Geometry::CYLINDER){
        urdf::Cylinder *cylinder = dynamic_cast<urdf::Cylinder*>(geometry);
	geom->type=2;
	geom->length=cylinder->length;
	geom->radius=cylinder->radius;
     }
     else if(geometry->type ==urdf::Geometry::SPHERE){
        urdf::Sphere *sphere = dynamic_cast<urdf::Sphere*>(geometry);
	geom->type=3;
	geom->radius=sphere->radius;
     }
  }
  int ConfigFile::processVisual(boost::shared_ptr<const urdf::Visual> visual,Link &link, int nmat, std::vector<Material> &materials){
     processGeometry(visual->geometry.get(),link.geom.get());
     processPose(visual->origin,link.position,link.rpy,link.quat);


     //Material
     //Search if it's a new materia
     int found=0;
     if(visual->material!=NULL){
       for(int i=0;i<nmat && !found;i++){
	 if(visual->material_name==materials[i].name){
	   link.material=i;
	   found=1;
	 }
       }

       if(!found){
	 materials[nmat].name=visual->material_name;
	 materials[nmat].r=visual->material->color.r;
	 materials[nmat].g=visual->material->color.g;
	 materials[nmat].b=visual->material->color.b;
	 materials[nmat].a=visual->material->color.a;
	 link.material=nmat;
	 nmat++;
       }
     }
     else
	link.material=-1;
     return nmat;
  }

  void ConfigFile::processJoint(boost::shared_ptr<const urdf::Joint> joint, Joint &jointVehicle,int parentLink,int childLink){
    jointVehicle.name=joint->name;
    jointVehicle.axis[0]=joint->axis.x;
    jointVehicle.axis[1]=joint->axis.y;
    jointVehicle.axis[2]=joint->axis.z;
    processPose(joint->parent_to_joint_origin_transform,jointVehicle.position,jointVehicle.rpy,jointVehicle.quat);
    jointVehicle.child=childLink;
    jointVehicle.parent=parentLink;

    if(joint->type==6)
	jointVehicle.type=0;
    else if(joint->type==1 || joint->type==2)
	jointVehicle.type=1;
    else if(joint->type==3)
	jointVehicle.type=2;
    else{
	OSG_WARN << "Unsupported type of joint in "<< joint->name << ", fixed joint will be used." << std::endl;
	jointVehicle.type=0;
    }

    //Mimic
    if(joint->mimic!=NULL){
      jointVehicle.mimic.reset(new Mimic);
      jointVehicle.mimic->jointName=joint->mimic->joint_name;
      jointVehicle.mimic->offset=joint->mimic->offset;
      jointVehicle.mimic->multiplier=joint->mimic->multiplier;
    }
    else
      jointVehicle.mimic.reset();

    //limits
    if(joint->limits!=NULL){
      jointVehicle.lowLimit=joint->limits->lower;
      jointVehicle.upLimit=joint->limits->upper;
    }
    else{
      jointVehicle.lowLimit=-M_PI;
      jointVehicle.upLimit=M_PI;
    }
  }

  int ConfigFile::processLink(boost::shared_ptr<const urdf::Link> link,Vehicle &vehicle,int nlink,int njoint,int nmat, std::vector<Material> &materials){
    vehicle.links[nlink].name=link->name;
    vehicle.links[nlink].geom.reset( new Geometry);
    if(link->visual)
      nmat= processVisual(link->visual,vehicle.links[nlink],nmat, materials);
    else{
      vehicle.links[nlink].geom->type=4;
      vehicle.links[nlink].material=-1;
      memset(vehicle.links[nlink].position,0,3*sizeof(double));
      memset(vehicle.links[nlink].rpy,0,3*sizeof(double));
      memset(vehicle.links[nlink].quat,0,4*sizeof(double));
      vehicle.links[nlink].quat[3]=1;
    }

    if(link->collision){
	vehicle.links[nlink].cs.reset( new Geometry);
	processGeometry(link->collision->geometry.get(),vehicle.links[nlink].cs.get());
    }
    else
	vehicle.links[nlink].cs.reset();

    int linkNumber=nlink;
    for(uint i=0;i<link->child_joints.size();i++){
       processJoint(link->child_joints[i],vehicle.joints[linkNumber],nlink,linkNumber+1);
       linkNumber=processLink(link->child_links[i],vehicle,linkNumber+1,linkNumber+1,nmat,materials);
    }
    return linkNumber;
  }

int ConfigFile::processURDFFile(string file, Vehicle &vehicle){
	urdf::Model model;

	std::string file_fullpath=osgDB::findDataFile(file);
	if (file_fullpath==std::string("") || !model.initFile(file_fullpath)){
		std::cerr << "Failed to parse urdf file " << file << std::endl;
		exit(0);
	}

	OSG_INFO << "Successfully parsed urdf file " << file << std::endl;

	vehicle.nlinks=model.links_.size();
	vehicle.links.resize(vehicle.nlinks);
	vehicle.njoints=model.joints_.size();
	vehicle.joints.resize(vehicle.njoints);
	vehicle.nmaterials = model.materials_.size();
	vehicle.materials.resize(vehicle.nmaterials);
	boost::shared_ptr<const urdf::Link> root = model.getRoot();
	processLink(root,vehicle,0,0,0,vehicle.materials);
	return 0;
}

void ConfigFile::processJointValues(const xmlpp::Node* node,std::vector<double> &jointValues, int &ninitJoints){
	xmlpp::Node::NodeList list = node->get_children();
	ninitJoints=(list.size()-1)/2;
	jointValues.resize((list.size()-1)/2);
	int pos=0;
	for(xmlpp::Node::NodeList::iterator iter = list.begin(); iter != list.end(); ++iter){
		xmlpp::Node* child=dynamic_cast<const xmlpp::Node*>(*iter);
		if(child->get_name()=="joint"){
			extractFloatChar(child,jointValues[pos++]);
		}
	}
}

void ConfigFile::postprocessVehicle(Vehicle &vehicle){
	//Get mimic parents
	int found;
	for(int i=0;i<vehicle.njoints;i++){
		if(vehicle.joints[i].mimic!=NULL){
			found=0;
			for(int j=0;j<vehicle.njoints && !found;j++){
				if(vehicle.joints[j].name==vehicle.joints[i].mimic->jointName){
					vehicle.joints[i].mimicp=j;
					found=1;
				}
			}
			if(found==0){
				vehicle.joints[i].mimicp=-1;
			}
		}
		else{
			vehicle.joints[i].mimicp=-1;
		}
	}

	//get camera joint
	Vcam aux;
	for(unsigned int i=0;i<vehicle.Vcams.size();i++){
		found=0;
		aux=vehicle.Vcams.front();
		vehicle.Vcams.pop_front();
		for(int j=0;j<vehicle.nlinks && !found;j++){
			if(vehicle.links[j].name==aux.linkName){
				aux.link=j;
				found=1;
			}
		}
		if(found==0) {
			OSG_WARN << "ConfigFile::postProcessVehicle: Camera attached to unknown link: " << aux.linkName << " camera will be ignored" << std::endl;
		} else
			vehicle.Vcams.push_back(aux);
	}

	//get range camera joint
	Vcam aux2;
	for(unsigned int i=0;i<vehicle.VRangecams.size();i++){
		found=0;
		aux2=vehicle.VRangecams.front();
		vehicle.VRangecams.pop_front();
		for(int j=0;j<vehicle.nlinks && !found;j++){
			if(vehicle.links[j].name==aux2.linkName){
				aux2.link=j;
				found=1;
			}
		}
		if(found==0) {
			OSG_WARN << "ConfigFile::postProcessVehicle: Range Camera attached to unknown link: " << aux.linkName << " camera will be ignored" << std::endl;
		} else
			vehicle.VRangecams.push_back(aux2);
	}

	//get Range sensor joint
	rangeSensor rs;
	for(unsigned int i=0;i<vehicle.range_sensors.size();i++){
		found=0;
		rs=vehicle.range_sensors.front();
		vehicle.range_sensors.pop_front();
		for(int j=0;j<vehicle.nlinks && !found;j++){
			if(vehicle.links[j].name==rs.linkName){
				rs.link=j;
				found=1;
			}
		}
		if(found==0) {
			OSG_WARN << "ConfigFile::postProcessVehicle: RangeSensor attached to unknown link: " << rs.linkName << ". Will be ignored"<< std::endl;
		} else
			vehicle.range_sensors.push_back(rs);
	}

	//get Imu joint
	Imu imu;
	for(unsigned int i=0;i<vehicle.imus.size();i++){
		found=0;
		imu=vehicle.imus.front();
		vehicle.imus.pop_front();
		for(int j=0;j<vehicle.nlinks && !found;j++){
			if(vehicle.links[j].name==imu.linkName){
				imu.link=j;
				found=1;
			}
		}
		if(found==0) {
			OSG_WARN << "ConfigFile::postProcessVehicle: IMU attached to unknown link: " << imu.linkName << ". Will be ignored"<< std::endl;
		} else
			vehicle.imus.push_back(imu);
	}

	//get PressureSensor joint
	XMLPressureSensor ps;
	for(unsigned int i=0;i<vehicle.pressure_sensors.size();i++){
		found=0;
		ps=vehicle.pressure_sensors.front();
		vehicle.pressure_sensors.pop_front();
		for(int j=0;j<vehicle.nlinks && !found;j++){
			if(vehicle.links[j].name==ps.linkName){
				ps.link=j;
				found=1;
			}
		}
		if(found==0) {
			OSG_WARN << "ConfigFile::postProcessVehicle: PressureSensor attached to unknown link: " << ps.linkName << ". Will be ignored"<< std::endl;
		} else
			vehicle.pressure_sensors.push_back(ps);
	}

	//get GPSSensor joint
	{
		XMLGPSSensor s;
		for(unsigned int i=0;i<vehicle.gps_sensors.size();i++){
			found=0;
			s=vehicle.gps_sensors.front();
			vehicle.gps_sensors.pop_front();
			for(int j=0;j<vehicle.nlinks && !found;j++){
				if(vehicle.links[j].name==s.linkName){
					s.link=j;
					found=1;
				}
			}
			if(found==0) {
				OSG_WARN << "ConfigFile::postProcessVehicle: GPSSensor attached to unknown link: " << s.linkName << ". Will be ignored"<< std::endl;
			} else
				vehicle.gps_sensors.push_back(s);
		}
	}

	//get DVLSensor joint
	{
		XMLDVLSensor s;
		for(unsigned int i=0;i<vehicle.dvl_sensors.size();i++){
			found=0;
			s=vehicle.dvl_sensors.front();
			vehicle.dvl_sensors.pop_front();
			for(int j=0;j<vehicle.nlinks && !found;j++){
				if(vehicle.links[j].name==s.linkName){
					s.link=j;
					found=1;
				}
			}
			if(found==0) {
				OSG_WARN << "ConfigFile::postProcessVehicle: DVLSensor attached to unknown link: " << s.linkName << ". Will be ignored"<< std::endl;
			} else
				vehicle.dvl_sensors.push_back(s);
		}
	}

	//get multibeamSensor joint
	{
		XMLMultibeamSensor MB;
		for(unsigned int i=0;i<vehicle.multibeam_sensors.size();i++){
			found=0;
			MB=vehicle.multibeam_sensors.front();
			vehicle.multibeam_sensors.pop_front();
			for(int j=0;j<vehicle.nlinks && !found;j++){
				if(vehicle.links[j].name==MB.linkName){
					MB.link=j;
					found=1;
				}
			}
			if(found==0) {
				OSG_WARN << "ConfigFile::postProcessVehicle: multibeamSensor attached to unknown link: " << MB.linkName << ". Will be ignored"<< std::endl;
			} else
				vehicle.multibeam_sensors.push_back(MB);
		}
	}

	//get Hand link
	for(unsigned int i=0;i<vehicle.object_pickers.size();i++){
		found=0;
		rs=vehicle.object_pickers.front();
		vehicle.object_pickers.pop_front();
		for(int j=0;j<vehicle.nlinks && !found;j++){
			if(vehicle.links[j].name==rs.linkName){
				rs.link=j;
				found=1;
			}
		}
		if(found==0) {
			OSG_WARN << "ObjectPicker attached to unknown link: "<< rs.linkName<<". Will be ignored" << std::endl;
		} else
			vehicle.object_pickers.push_back(rs);
	}
}

void ConfigFile::processVehicle(const xmlpp::Node* node,Vehicle &vehicle){
	xmlpp::Node::NodeList list = node->get_children();
	for(xmlpp::Node::NodeList::iterator iter = list.begin(); iter != list.end(); ++iter){
		xmlpp::Node* child=dynamic_cast<const xmlpp::Node*>(*iter);

		if(child->get_name()=="name")
			extractStringChar(child,vehicle.name);
		else if(child->get_name()=="file"){
			string aux;
			extractStringChar(child,aux);
			processURDFFile(aux,vehicle);
		}
		else if(child->get_name()=="position")
			extractPositionOrColor(child,vehicle.position);
		else if(child->get_name()=="orientation")
			extractOrientation(child,vehicle.orientation);
		else if (child->get_name()=="jointValues")
			processJointValues(child,vehicle.jointValues,vehicle.ninitJoints);
		else if (child->get_name()=="virtualCamera"){
			Vcam aux;
			aux.init();
			processVcam(child,aux);
			vehicle.Vcams.push_back(aux);
		}
		else if (child->get_name()=="virtualRangeImage"){
			Vcam aux;
			aux.init();
			aux.range=1;
			processVcam(child,aux);
			vehicle.VRangecams.push_back(aux);
		} else if (child->get_name()=="rangeSensor"){
			rangeSensor aux;
			aux.init();
			processRangeSensor(child,aux);
			vehicle.range_sensors.push_back(aux);
		} else if (child->get_name()=="objectPicker"){
			rangeSensor aux;
			aux.init();
			processRangeSensor(child,aux);
			vehicle.object_pickers.push_back(aux);
		} else if (child->get_name()=="imu"){
			Imu aux;
			aux.init();
			processImu(child,aux);
			vehicle.imus.push_back(aux);
		} else if (child->get_name()=="pressureSensor"){
			XMLPressureSensor aux;
			aux.init();
			processPressureSensor(child,aux);
			vehicle.pressure_sensors.push_back(aux);
		} else if (child->get_name()=="gpsSensor"){
			XMLGPSSensor aux;
			aux.init();
			processGPSSensor(child,aux);
			vehicle.gps_sensors.push_back(aux);
		} else if (child->get_name()=="dvlSensor"){
			XMLDVLSensor aux;
			aux.init();
			processDVLSensor(child,aux);
			vehicle.dvl_sensors.push_back(aux);
		} else if (child->get_name()=="multibeamSensor"){
			XMLMultibeamSensor aux;
			aux.init();
			processMultibeamSensor(child,aux);
			vehicle.multibeam_sensors.push_back(aux);
		} else if (child->get_name()=="simulatedDevices"){
			std::vector< SimulatedDeviceConfig::Ptr > auxs = SimulatedDevices::processConfig(child, this);
			for (size_t i=0; i< auxs.size(); ++i)
				if (auxs[i])
					vehicle.simulated_devices.push_back(auxs[i]);
		}
	}
}

void ConfigFile::processPhysicProperties(const xmlpp::Node* node, PhysicProperties &pp){
	xmlpp::Node::NodeList list = node->get_children();
	for(xmlpp::Node::NodeList::iterator iter = list.begin(); iter != list.end(); ++iter){
		xmlpp::Node* child=dynamic_cast<const xmlpp::Node*>(*iter);
		
		if(child->get_name()=="mass")
			extractFloatChar(child,pp.mass);
		else if(child->get_name()=="inertia")
			extractPositionOrColor(child,pp.inertia);
		else if(child->get_name()=="collisionShapeType")
			extractStringChar(child,pp.csType);
		else if(child->get_name()=="linearDamping"){
			extractFloatChar(child,pp.linearDamping);		
			if(pp.linearDamping>1.0)
			  OSG_WARN << "ConfigFile::PhysicProperties: linearDamping is higher than 1.0."<< std::endl;
		}
		else if(child->get_name()=="angularDamping"){
			extractFloatChar(child,pp.angularDamping);		
			if(pp.linearDamping>1.0)
			  OSG_WARN << "ConfigFile::PhysicProperties: angularDamping is higher than 1.0."<< std::endl;
		}
		else if(child->get_name()=="minLinearLimit")
			extractPositionOrColor(child,pp.minLinearLimit);
		else if(child->get_name()=="maxLinearLimit")
			extractPositionOrColor(child,pp.maxLinearLimit);
		else if(child->get_name()=="isKinematic"){
			extractIntChar(child,pp.isKinematic);
			if (pp.isKinematic!=0 && pp.isKinematic!=1) {
				OSG_WARN <<"ConfigFile::PhysicProperties: isKinematic is not a binary value ( 0 1), using default value (0)" << std::endl;
				freeMotion=0;
			}
		}
		else if(child->get_name()=="minAngularLimit")
			extractPositionOrColor(child,pp.minAngularLimit);
		else if(child->get_name()=="maxAngularLimit")
			extractPositionOrColor(child,pp.maxAngularLimit);
		
	}
}

void ConfigFile::processObject(const xmlpp::Node* node,Object &object){
	xmlpp::Node::NodeList list = node->get_children();
	for(xmlpp::Node::NodeList::iterator iter = list.begin(); iter != list.end(); ++iter){
		xmlpp::Node* child=dynamic_cast<const xmlpp::Node*>(*iter);

		if(child->get_name()=="name")
			extractStringChar(child,object.name);
		else if(child->get_name()=="file")
			extractStringChar(child,object.file);
		else if(child->get_name()=="position")
			extractPositionOrColor(child,object.position);
		else if(child->get_name()=="orientation")
			extractOrientation(child,object.orientation);
		else if(child->get_name()=="offsetp")
			extractPositionOrColor(child,object.offsetp);
		else if(child->get_name()=="offsetr")
			extractPositionOrColor(child,object.offsetr);
		else if(child->get_name()=="physics"){
                        object.physicProperties.reset(new PhysicProperties);
			object.physicProperties->init();
			processPhysicProperties(child,*object.physicProperties);

		}

	}
}

void ConfigFile::processROSInterface(const xmlpp::Node* node,ROSInterfaceInfo &rosInterface){
	xmlpp::Node::NodeList list = node->get_children();
	for(xmlpp::Node::NodeList::iterator iter = list.begin(); iter != list.end(); ++iter){
		xmlpp::Node* child=dynamic_cast<const xmlpp::Node*>(*iter);
		if(child->get_name()!="text"){//adding all nodes as text for further processing by a SimulatedDevice
			std::string text;
			extractStringChar(child,text);
			rosInterface.values[child->get_name()] = text;
		}
		
		if(child->get_name()=="topic" || child->get_name()=="imageTopic" )
			extractStringChar(child,rosInterface.topic);
		else if(child->get_name()=="vehicleName" || child->get_name()=="cameraName" || child->get_name()=="name")
			extractStringChar(child,rosInterface.targetName);
		else if(child->get_name()=="rate")
			extractIntChar(child,rosInterface.rate);
		else if(child->get_name()=="infoTopic")
			extractStringChar(child,rosInterface.infoTopic);
		else if(child->get_name()=="width")
			extractUIntChar(child,rosInterface.w);
		else if(child->get_name()=="height")
			extractUIntChar(child,rosInterface.h);
		else if(child->get_name()=="posx")
			extractUIntChar(child,rosInterface.posx);
		else if(child->get_name()=="posy")
			extractUIntChar(child,rosInterface.posy);
		else if(child->get_name()=="blackWhite"){
			extractUIntChar(child,rosInterface.blackWhite);
			if (rosInterface.blackWhite!=0 && rosInterface.blackWhite!=1) {
				OSG_WARN <<"ConfigFile::processROSInterface: blackWhite is not a binary value ( 0 1), using default value (0)" << std::endl;
				rosInterface.blackWhite=0;
			}
		}else if(child->get_name()=="depth"){
			extractUIntChar(child,rosInterface.depth);
			if (rosInterface.depth!=0 && rosInterface.depth!=1) {				
				osg::notify(osg::ALWAYS)  <<"ConfigFile::processROSInterface: depth is not a binary value ( 0 1), using default value (0)" << std::endl;
				rosInterface.depth=0;
			}
		}else if(child->get_name()=="scale")
			extractFloatChar(child,rosInterface.scale);
		else if(child->get_name()=="visualize")
			extractIntChar(child,rosInterface.visualize);
		else if(child->get_name()=="color")
			extractPositionOrColor(child,rosInterface.color);
		else if(child->get_name()=="text"){}//we ignore this as whitespace is treated as a whole child element}    
		else
			osg::notify(osg::ALWAYS)  <<"processROSInterface:: unexpected child: " << child->get_name() <<std::endl;			
	}
}

void ConfigFile::processROSInterfaces(const xmlpp::Node* node){
	xmlpp::Node::NodeList list = node->get_children();
	for(xmlpp::Node::NodeList::iterator iter = list.begin(); iter != list.end(); ++iter){
		xmlpp::Node* child=dynamic_cast<const xmlpp::Node*>(*iter);
		xmlpp::Node* subchild=NULL;

		ROSInterfaceInfo rosInterface;
		rosInterface.rate=10; //Default rate
		rosInterface.posx=rosInterface.posy=0;
		rosInterface.scale=1;
		rosInterface.visualize=0;
		rosInterface.type=ROSInterfaceInfo::Unknown;
		rosInterface.depth=0;
		if(child->get_name()=="ROSOdomToPAT"){
			rosInterface.type=ROSInterfaceInfo::ROSOdomToPAT;
			rosInterface.color[0]=rosInterface.color[1]=rosInterface.color[2]=1;
		} else if(child->get_name()=="PATToROSOdom"){
			rosInterface.type=ROSInterfaceInfo::PATToROSOdom;
		} else if(child->get_name()=="ArmToROSJointState") {
			rosInterface.type=ROSInterfaceInfo::ArmToROSJointState;
		} else if(child->get_name()=="ROSJointStateToArm") {
			rosInterface.type=ROSInterfaceInfo::ROSJointStateToArm;
		} else if(child->get_name()=="VirtualCameraToROSImage") {
			rosInterface.type=ROSInterfaceInfo::VirtualCameraToROSImage;
		} else if(child->get_name()=="RangeImageSensorToROSImage") {
			rosInterface.type=ROSInterfaceInfo::RangeImageSensorToROSImage;
		} else if(child->get_name()=="RangeSensorToROSRange") {
			rosInterface.type=ROSInterfaceInfo::RangeSensorToROSRange;
		} else if(child->get_name()=="ROSImageToHUD") {
			rosInterface.type=ROSInterfaceInfo::ROSImageToHUD;
		} else if(child->get_name()=="ROSTwistToPAT"){
			rosInterface.type=ROSInterfaceInfo::ROSTwistToPAT;
		} else if(child->get_name()=="ROSPoseToPAT"){
			rosInterface.type=ROSInterfaceInfo::ROSPoseToPAT;
		} else if(child->get_name()=="ImuToROSImu"){
			rosInterface.type=ROSInterfaceInfo::ImuToROSImu;
		} else if(child->get_name()=="PressureSensorToROS"){
			rosInterface.type=ROSInterfaceInfo::PressureSensorToROS;
		} else if(child->get_name()=="GPSSensorToROS"){
			rosInterface.type=ROSInterfaceInfo::GPSSensorToROS;
		} else if(child->get_name()=="DVLSensorToROS"){
			rosInterface.type=ROSInterfaceInfo::DVLSensorToROS;
		} else if(child->get_name()=="multibeamSensorToLaserScan"){
			rosInterface.type=ROSInterfaceInfo::multibeamSensorToLaserScan;
		} else if(child->get_name()=="SimulatedDeviceROS"){
			xmlpp::Node::NodeList sublist = child->get_children();
				for(xmlpp::Node::NodeList::iterator subiter = sublist.begin(); subiter != sublist.end(); ++subiter){
					xmlpp::Node* tempchild=dynamic_cast<const xmlpp::Node*>(*subiter);
					if (tempchild!=NULL && tempchild->get_name()!="text"){
						std::string subtype = tempchild->get_name();
						//std::cout<< "subtype="<<subtype<<std::endl;
						if (subtype.length()>3 && subtype.substr(subtype.length()-3,3)=="ROS"){
							rosInterface.type=ROSInterfaceInfo::SimulatedDevice;
							rosInterface.subtype = subtype.substr(0, subtype.length()-3);
							subchild = tempchild;
						}
					}
				}
		}

		if (rosInterface.type!=ROSInterfaceInfo::Unknown) {
			processROSInterface(subchild!=NULL ? subchild : child, rosInterface);
			ROSInterfaces.push_back(rosInterface);
		}
	}
}

void ConfigFile::processXML(const xmlpp::Node* node){
	if(node->get_name()!="UWSimScene") {
		OSG_WARN <<"ConfigFile::processXML: XML file is not an UWSimScene file."<< std::endl;
	} else{
		xmlpp::Node::NodeList list = node->get_children();
		for(xmlpp::Node::NodeList::iterator iter = list.begin(); iter != list.end(); ++iter)
		{
			xmlpp::Node* child=dynamic_cast<const xmlpp::Node*>(*iter);
			if(child->get_name()=="oceanState")
				processOceanState(child);
			else if(child->get_name()=="simParams")
				processSimParams(child);
			else if(child->get_name()=="camera")
				processCamera(child);
			else if(child->get_name()=="vehicle"){
				Vehicle vehicle;
				processVehicle(child,vehicle);
				postprocessVehicle(vehicle);
				vehicles.push_back(vehicle);
			}
			else if(child->get_name()=="object"){
				Object  object;
				memset(object.offsetp,0,3*sizeof(double));
				memset(object.offsetr,0,3*sizeof(double));
				object.physicProperties.reset();
				processObject(child,object);
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
   	memset(gravity,0,3*sizeof(double));
	camNear=camFar=-1;
        enablePhysics=0;
	try
	{
		xmlpp::DomParser parser;
		parser.set_validate();
		parser.set_substitute_entities(); //We just want the text to be resolved/unescaped automatically.
		std::string fName_fullpath=osgDB::findDataFile(fName);
		if (fName_fullpath!=std::string("")) {
			parser.parse_file(fName_fullpath);
			if(parser)
			{
				//Walk the tree:
				const xmlpp::Node* pNode = parser.get_document()->get_root_node(); //deleted by DomParser.
				processXML(pNode);
			}
		} else {
			std::cerr << "Cannot locate file " << fName <<  std::endl;
			exit(0);
		}
	}
	catch(const std::exception& ex)
	{
		std::cerr << "Exception caught: " << ex.what() << std::endl;
		std::cerr << "Please check your XML file" << std::endl;
		exit(0);
	}

}
