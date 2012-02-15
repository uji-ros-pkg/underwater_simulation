#include "ROSInterface.h"

ROSSubscriberInterface::ROSSubscriberInterface(std::string topic):ROSInterface(topic) {
  OSG_DEBUG << "ROSSubscriberVehicleInterface Thread starting... " << topic << std::endl;
  startThread();
  OSG_DEBUG << "ROSSubscriberVehicleInterface Thread created" << std::endl;
}

/* Thread code */
void ROSSubscriberInterface::run() {
  //int argc=0;
  //char **argv=NULL;

  std::string nodeName=topic;
  std::replace( nodeName.begin(), nodeName.end(), '/', '_' );
  //ros::init(argc,argv,"UWSim_Sub_"+nodeName);
  ros::NodeHandle nh_;

  //FIXME: Corregir condiciones de carrera
  //small sleep to let processData to be settled by the child classes
  ros::Duration(4.).sleep();

  createSubscriber(nh_);		
  ros::spin();
}

ROSSubscriberInterface::~ROSSubscriberInterface(){}


ROSOdomToPAT::ROSOdomToPAT(osg::Group *rootNode, std::string topic, std::string vehicleName): ROSSubscriberInterface(topic) {
  findNodeVisitor findNode(vehicleName);
  rootNode->accept(findNode);
  osg::Node *first=findNode.getFirst();
  if (first==NULL) {
    transform=NULL;
  } else {
    transform=dynamic_cast<osg::MatrixTransform*>(first);
  }
  started=0; //Used in time
}

void ROSOdomToPAT::createSubscriber(ros::NodeHandle &nh) {
  sub_ = nh.subscribe<nav_msgs::Odometry>(topic, 10, &ROSOdomToPAT::processData, this);
}

void ROSOdomToPAT::processData(const nav_msgs::Odometry::ConstPtr& odom) {
  if (transform!=NULL) {
    //vpHomogeneousMatrix sMsv;
    osg::Matrixd sMsv_osg;
    //If velocity is zero, use the position reference. If not, use velocity
    if (odom->twist.twist.angular.x==0 && odom->twist.twist.angular.y==0 && odom->twist.twist.angular.z==0 &&
  	odom->twist.twist.linear.x==0 && odom->twist.twist.linear.y==0 && odom->twist.twist.linear.z==0) {
      sMsv_osg.setRotate(osg::Quat(odom->pose.pose.orientation.x, odom->pose.pose.orientation.y, odom->pose.pose.orientation.z, odom->pose.pose.orientation.w));
      sMsv_osg.setTrans(odom->pose.pose.position.x,odom->pose.pose.position.y,odom->pose.pose.position.z);
    } else {
      //Get the current vehicle position and attitude in an homogeneous matrix
      sMsv_osg=transform->getMatrix();
			
      //Time issues
      double elapsed=0;
      if (started!=0) {
        ros::WallDuration t_diff = ros::WallTime::now() - last;
        elapsed= t_diff.toSec();
        //If elapsed>MAX_ELAPSED, consider this is sent by a different publisher, so that the counter has to restart
        if (elapsed>1) elapsed=0;
      }	
      started=1;
      last = ros::WallTime::now();
      //std::cout<<elapsed<<std::endl;

      osg::Matrixd vMnv;
      osg::Matrixd T, Rx, Ry, Rz;
      T.makeTranslate(odom->twist.twist.linear.x*elapsed,odom->twist.twist.linear.y*elapsed,odom->twist.twist.linear.z*elapsed);
      Rx.makeRotate(odom->twist.twist.angular.x*elapsed,1,0,0);
      Ry.makeRotate(odom->twist.twist.angular.y*elapsed,0,1,0);
      Rz.makeRotate(odom->twist.twist.angular.z*elapsed,0,0,1);
      vMnv=Rz*Ry*Rx*T;

      sMsv_osg=vMnv*sMsv_osg;
    }
    transform->setMatrix(sMsv_osg);
  }
}

ROSOdomToPAT::~ROSOdomToPAT(){}

ROSTwistToPAT::ROSTwistToPAT(osg::Group *rootNode, std::string topic, std::string vehicleName): ROSSubscriberInterface(topic) {
  findNodeVisitor findNode(vehicleName);
  rootNode->accept(findNode);
  osg::Node *first=findNode.getFirst();
  if (first==NULL) {
    transform=NULL;
  } else {
    transform=dynamic_cast<osg::MatrixTransform*>(first);
  }
  started=0; //used in time
}

void ROSTwistToPAT::createSubscriber(ros::NodeHandle &nh) {
  sub_ = nh.subscribe<geometry_msgs::TwistStamped>(topic, 10, &ROSTwistToPAT::processData, this);
}

void ROSTwistToPAT::processData(const geometry_msgs::TwistStamped::ConstPtr& twist) {
  if (transform!=NULL) {
    //vpHomogeneousMatrix sMsv;
    osg::Matrixd sMsv_osg;
    //Get the current vehicle position and attitude in an homogeneous matrix
    sMsv_osg=transform->getMatrix();
			
    //Time issues
    double elapsed=0;
    if (started!=0) {
      ros::WallDuration t_diff = ros::WallTime::now() - last;
      elapsed= t_diff.toSec();
      //If elapsed>MAX_ELAPSED, consider this is sent by a different publisher, so that the counter has to restart
      if (elapsed>1) elapsed=0;
    }	
    started=1;
    last = ros::WallTime::now();

    osg::Matrixd vMnv;
    osg::Matrixd T, Rx, Ry, Rz;
    T.makeTranslate(twist->twist.linear.x*elapsed,twist->twist.linear.y*elapsed,twist->twist.linear.z*elapsed);
    Rx.makeRotate(twist->twist.angular.x*elapsed,1,0,0);
    Ry.makeRotate(twist->twist.angular.y*elapsed,0,1,0);
    Rz.makeRotate(twist->twist.angular.z*elapsed,0,0,1);
    vMnv=Rz*Ry*Rx*T;
      
    sMsv_osg=vMnv*sMsv_osg;
  
    transform->setMatrix(sMsv_osg);
  }
}

ROSTwistToPAT::~ROSTwistToPAT(){}

/*
class ROSNavigationDataToPAT: public ROSSubscriberInterface {
	osg::PositionAttitudeTransform *transform;
public:
	ROSNavigationDataToPAT(std::string topic, osg::PositionAttitudeTransform *t): ROSSubscriberInterface(topic) {
		transform=t;
	}

	virtual void createSubscriber(ros::NodeHandle &nh) {
		sub_ = nh.subscribe<cola2_common::NavigationData>(topic, 10, &ROSNavigationDataToPAT::processData, this);
	}

	virtual void processData(const cola2_common::NavigationData::ConstPtr& odom) {
		//Simulated vehicle frame wrt real vehicle frame
		vpHomogeneousMatrix vMsv(0.8,0,0.8,0,M_PI,0);
	
		vpHomogeneousMatrix sMsv;
	

			//Set a position reference
			//Pose of the real vehicle wrt to the localization origin
			vpRotationMatrix pRv(vpRxyzVector(odom->pose[3],odom->pose[4],odom->pose[5]));
			vpTranslationVector pTv(odom->pose[0],odom->pose[1],odom->pose[2]);
			vpHomogeneousMatrix pMv;
			pMv.buildFrom(pTv, pRv);

			//Localization origin wrt simulator origin
			vpRxyzVector sRVp(M_PI,0,-M_PI_2);
			vpRotationMatrix sRp(sRVp);
			vpTranslationVector sTp(-514921,-4677958,3.4);

			vpHomogeneousMatrix sMp;
			sMp.buildFrom(sTp,sRp);

			sMsv=sMp*pMv*vMsv;
		
		if (transform!=NULL) {
			//OSG_DEBUG << "SimulatedVehicle::processData baseTransform not null" << std::endl;
	 		transform->setPosition(osg::Vec3d(sMsv[0][3],sMsv[1][3],sMsv[2][3]));
			vpRotationMatrix mr;
			sMsv.extract(mr);
			vpRxyzVector vr(mr);
			osg::Quat sQsv(vr[0],osg::Vec3d(1,0,0), vr[1],osg::Vec3d(0,1,0), vr[2],osg::Vec3d(0,0,1));
			transform->setAttitude(sQsv);
			//baseTransform->setAttitude(osg::Quat(js->pose.pose.orientation.w,osg::Vec3d(0,0,1)));
		}
	}

	~ROSNavigationDataToPAT(){}
};
*/


ROSJointStateToArm::ROSJointStateToArm(std::string topic, SimulatedIAUV *arm): ROSSubscriberInterface(topic) {
  this->arm=arm;
}

void ROSJointStateToArm::createSubscriber(ros::NodeHandle &nh) {
  sub_ = nh.subscribe<sensor_msgs::JointState>(topic, 10, &ROSJointStateToArm::processData, this);
}

void ROSJointStateToArm::processData(const sensor_msgs::JointState::ConstPtr& js) {
  //Receive request from client
  if (js->position.size()!=0) {
    //position command
    double *q = new double [js->position.size()];
    for (unsigned int i=0; i<js->position.size(); i++) {
      q[i]=js->position[i];
    }
    arm->urdf->setJointPosition(q);
    delete [] q;
  } else if (js->velocity.size()!=0) {
    //velocity command
    double *qdot = new double [js->velocity.size()];
    for (unsigned int i=0; i<js->velocity.size(); i++) {
      qdot[i]=js->velocity[i];
    }
    arm->urdf->setJointVelocity(qdot);
    delete [] qdot;
  }
}

ROSJointStateToArm::~ROSJointStateToArm(){}


ROSImageToHUDCamera::ROSImageToHUDCamera(std::string image_topic, std::string info_topic, HUDCamera *cam): ROSSubscriberInterface(info_topic) {
  this->cam=cam;
  this->image_topic=image_topic;
}

void ROSImageToHUDCamera::createSubscriber(ros::NodeHandle &nh) {
  it=new image_transport::ImageTransport(nh);
  OSG_DEBUG << "ROSImageToHUDCamera::createSubscriber Subscribing to image topic " << image_topic << std::endl;
  image_sub=it->subscribe(image_topic, 1, &ROSImageToHUDCamera::processData, this);	
  //OSG_INFO << "ROSCamera::ROSCamera Subscribing to camera info topic " << info_topic << std::endl;
  //sub_=nh_.subscribe<sensor_msgs::CameraInfo>(info_topic, 1, &ROSImageToHUDCamera::imageInfoCallback, this);
}

void ROSImageToHUDCamera::processData(const sensor_msgs::ImageConstPtr& msg) {
  OSG_DEBUG << "ROSImageToHUDCamera::imageCallback start: " << msg->width << "x" << msg->height << " step:" << msg->step << std::endl;
	
  //unsigned char *msgdata=(unsigned char*)&(msg->data[0]);
  //osg_image->setImage(width,height,1,0,GL_RGB,GL_UNSIGNED_BYTE,msgdata,osg::Image::NO_DELETE);
  char *osgimage_data=(char*)cam->osg_image->data();
  //Memory cannot be directly copied, since the image frame used in OpenSceneGraph (OpenGL glReadPixels) is on
  //the bottom-left looking towards up-right, whereas ROS sensor_msgs::Image::data expects origin on top-left
  //looking towards bottom-right. Therefore it must be manually arranged, although this could be much improved:
  for (unsigned int i=0; i<msg->height; i++) {
    for (unsigned int j=0; j<msg->step; j++) {
      osgimage_data[i*msg->step+j]=msg->data[(msg->height-i-1)*msg->step+j];
    }
  }
  cam->ready_=true;
  OSG_DEBUG << "ROSImageToHUDCamera::imageCallback exit" << std::endl;
}

ROSImageToHUDCamera::~ROSImageToHUDCamera(){}



ROSPublisherInterface::ROSPublisherInterface(std::string topic, int publish_rate): ROSInterface(topic) {
  this->publish_rate=publish_rate;	
  OSG_DEBUG << "ROSPublisherInterface Thread starting..." << topic  << std::endl;
  startThread();
  OSG_DEBUG << "ROSPublisherInterface Thread created" << std::endl;
}

/* Thread code */
void ROSPublisherInterface::run() {
  //int argc=0;
  //char **argv=NULL;

  std::string nodeName=topic;
  std::replace( nodeName.begin(), nodeName.end(), '/', '_' );
  //ros::init(argc,argv,"UWSim_Pub_"+nodeName);
  ros::NodeHandle nh_;

  //FIXME: Corregir condiciones de carrera
  //small sleep to let processData to be settled by the child classes
  ros::Duration(4.).sleep();

  createPublisher(nh_);

  ros::Rate rate(publish_rate);
  while (ros::ok()) {
   publish();

   ros::spinOnce();
   rate.sleep();
  }
}

ROSPublisherInterface::~ROSPublisherInterface(){}


PATToROSOdom::PATToROSOdom(osg::Group *rootNode,std::string vehicleName, std::string topic, int rate): ROSPublisherInterface(topic,rate) {

  findNodeVisitor findNode(vehicleName);
  rootNode->accept(findNode);
  osg::Node *first=findNode.getFirst();
  if (first==NULL) {
    transform=NULL;
  } else {
    transform=dynamic_cast<osg::MatrixTransform*>(first);
  }
}

void PATToROSOdom::createPublisher(ros::NodeHandle &nh) {
  pub_ = nh.advertise<nav_msgs::Odometry>(topic, 1);
}

void PATToROSOdom::publish() {
  if (transform!=NULL) {
    nav_msgs::Odometry odom;
	
    osg::Matrixd mat=transform->getMatrix();
    osg::Vec3d pos=mat.getTrans();
    osg::Quat rot=mat.getRotate();

    odom.pose.pose.position.x=pos.x();
    odom.pose.pose.position.y=pos.y();
    odom.pose.pose.position.z=pos.z();
    odom.pose.pose.orientation.x=rot.x();
    odom.pose.pose.orientation.y=rot.y();
    odom.pose.pose.orientation.z=rot.z();
    odom.pose.pose.orientation.w=rot.w();

    //twist and covariances not implemented at the moment
    odom.twist.twist.linear.x=0;
    odom.twist.twist.linear.y=0;
    odom.twist.twist.linear.z=0;
    odom.twist.twist.angular.x=0;
    odom.twist.twist.angular.y=0;
    odom.twist.twist.angular.z=0;
    for (int i=0; i<36; i++) {
      odom.twist.covariance[i]=0;
      odom.pose.covariance[i]=0;
    }

    pub_.publish(odom);
  }
}
	
PATToROSOdom::~PATToROSOdom() {}


ArmToROSJointState::ArmToROSJointState(SimulatedIAUV *arm, std::string topic, int rate): ROSPublisherInterface(topic,rate) {
  this->arm=arm->urdf;
}

void ArmToROSJointState::createPublisher(ros::NodeHandle &nh) {
  pub_ = nh.advertise<sensor_msgs::JointState>(topic, 1);
}

void ArmToROSJointState::publish() {
  if (arm!=NULL) {
    sensor_msgs::JointState js;
    std::vector<double> q=arm->getJointPosition();
    for (size_t i=0; i<q.size(); i++) {
      char name[4];
      sprintf(name,"q%d",i+1);
      js.name.push_back(std::string(name));
      js.position.push_back(q[i]);
      js.effort.push_back(0);
    }
	
    pub_.publish(js);
  }
}
	
ArmToROSJointState::~ArmToROSJointState() {}
	

VirtualCameraToROSImage::VirtualCameraToROSImage(VirtualCamera *cam, std::string topic, std::string info_topic, int rate): ROSPublisherInterface	(info_topic,rate) {
  this->cam=cam;
  image_topic=topic;
}

void VirtualCameraToROSImage::createPublisher(ros::NodeHandle &nh) {
  it.reset(new image_transport::ImageTransport(nh));
  img_pub_ = it->advertise(image_topic, 1);
  pub_=nh.advertise<sensor_msgs::CameraInfo>(topic, 1);
}

void VirtualCameraToROSImage::publish() {
  //OSG_DEBUG << "OSGImageToROSImage::publish entering" << std::endl;
  if (cam->renderTexture!=NULL && cam->renderTexture->getTotalSizeInBytes()!=0) {
    //OSG_DEBUG << "\t image size: " << cam->renderTexture->s() << " " << cam->renderTexture->t() << " " << cam->renderTexture->getTotalSizeInBytes() << std::endl;
    int w, h, d;
    w=cam->renderTexture->s();
    h=cam->renderTexture->t();
    d=cam->renderTexture->getTotalSizeInBytes();

    if (d!=0) {
      sensor_msgs::Image img;
      sensor_msgs::CameraInfo img_info;
      img_info.header.stamp=img.header.stamp=ros::Time::now();
      img.encoding=std::string("rgb8");
      img.is_bigendian=0;
      img.height=h;
      img.width=w;
      img.step=d/h;
      img.data.resize(d);
      img_info.width=w;
      img_info.height=h;
      img_info.K[0]=cam->fx;
      img_info.K[2]=cam->cx;
      img_info.K[4]=cam->fy;
      img_info.K[5]=cam->cy;
      img_info.K[8]=1;

      img_info.P[0]=cam->fx;
      img_info.P[2]=cam->cx;
      img_info.P[5]=cam->fy;
      img_info.P[6]=cam->cy;
      img_info.P[10]=1;
 
      img_info.distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;		
	
      char *virtualdata=(char*)cam->renderTexture->data();
      //memcpy(&(img.data.front()),virtualdata,d*sizeof(char));
      //Memory cannot be directly copied, since the image frame used in OpenSceneGraph (OpenGL glReadPixels) is on
      //the bottom-left looking towards up-right, whereas ROS sensor_msgs::Image::data expects origin on top-left
      //looking towards bottom-right. Therefore it must be manually arranged, although this could be much improved:
      for (int i=0; i<h; i++) {
        for (unsigned int j=0; j<img.step; j++) {
          img.data[(h-i-1)*img.step+j]=virtualdata[i*img.step+j];
	}
      }
      img_pub_.publish(img);
      pub_.publish(img_info);
    }
  } 
  //OSG_DEBUG << "OSGImageToROSImage::publish exit" << std::endl;		
}
	
VirtualCameraToROSImage::~VirtualCameraToROSImage() {}



RangeSensorToROSRange::RangeSensorToROSRange(VirtualRangeSensor *rs, std::string topic, int rate): ROSPublisherInterface(topic,rate) {
  this->rs=rs;
}

void RangeSensorToROSRange::createPublisher(ros::NodeHandle &nh) {
  pub_ = nh.advertise<sensor_msgs::Range>(topic, 1);
}

void RangeSensorToROSRange::publish() {
  if (rs!=NULL) {
    sensor_msgs::Range r;
    r.radiation_type=sensor_msgs::Range::ULTRASOUND;
    r.field_of_view=0;	//X axis of the sensor
    r.min_range=0;
    r.max_range=rs->range;
    r.range=rs->node_tracker->distance_to_obstacle;
	
    pub_.publish(r);
  }
}
	
RangeSensorToROSRange::~RangeSensorToROSRange() {}
