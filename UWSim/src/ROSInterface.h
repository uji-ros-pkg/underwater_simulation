/*
 * ROSInterface.h
 * Classes for UWSim - ROS communication
 *
 *  Created on: 16/05/2011
 *      Author: mprats
 */


#ifndef ROSINTERFACE_H_
#define ROSINTERFACE_H_

//UWSIM
#include "SimulatorConfig.h"

#ifdef BUILD_ROS_INTERFACES

#include "URDFRobot.h"
#include "SimulatedIAUV.h"
#include "VirtualCamera.h"
#include "VirtualRangeSensor.h"
#include "HUDCamera.h"
#include "UWSimUtils.h"

//OSG
#include <OpenThreads/Thread>

//STL
#include <vector>

#include <boost/shared_ptr.hpp>

//ROS
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/distortion_models.h>
#include <sensor_msgs/Range.h>
#include <image_transport/image_transport.h>
#include <geometry_msgs/TwistStamped.h>
//#include <cola2_common/NavigationData.h>

//Max time (in seconds) between two consecutive control references
#define MAX_ELAPSED	1

class ROSInterface {
protected:
	std::string topic;
	ros::NodeHandle nh_;
public:
	ROSInterface(std::string topic) {this->topic=topic;}

	~ROSInterface(){}
};

class ROSSubscriberInterface: public OpenThreads::Thread, public ROSInterface {
protected:
	ros::Subscriber sub_;
public:
	ROSSubscriberInterface(std::string topic);

	virtual void createSubscriber(ros::NodeHandle &nh)=0;

	/* Thread code */
	void run();

	~ROSSubscriberInterface();
};

class ROSOdomToPAT: public ROSSubscriberInterface {
	osg::ref_ptr<osg::MatrixTransform> transform;
        ros::WallTime last;
        int started;

public:
	ROSOdomToPAT(osg::Group *rootNode, std::string topic, std::string vehicleName);

	virtual void createSubscriber(ros::NodeHandle &nh);

	virtual void processData(const nav_msgs::Odometry::ConstPtr& odom);
	~ROSOdomToPAT();
};

class ROSTwistToPAT: public ROSSubscriberInterface {
	osg::ref_ptr<osg::MatrixTransform> transform;
	ros::WallTime last;
	int started;
public:
	ROSTwistToPAT(osg::Group *rootNode, std::string topic, std::string vehicleName);

	virtual void createSubscriber(ros::NodeHandle &nh);

	virtual void processData(const geometry_msgs::TwistStamped::ConstPtr& odom);
	~ROSTwistToPAT();
};

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


class ROSJointStateToArm: public ROSSubscriberInterface {
	boost::shared_ptr<SimulatedIAUV> arm;
public:
	ROSJointStateToArm(std::string topic, boost::shared_ptr<SimulatedIAUV> arm);
	virtual void createSubscriber(ros::NodeHandle &nh);

	virtual void processData(const sensor_msgs::JointState::ConstPtr& js);
	~ROSJointStateToArm();
};

class ROSImageToHUDCamera: public ROSSubscriberInterface {
	boost::shared_ptr<HUDCamera> cam;
	boost::shared_ptr<image_transport::ImageTransport> it;
	image_transport::Subscriber image_sub;
	std::string image_topic;
public:
	ROSImageToHUDCamera(std::string topic, std::string info_topic, HUDCamera *camera);

	virtual void createSubscriber(ros::NodeHandle &nh);

	virtual void processData(const sensor_msgs::ImageConstPtr& msg);
	~ROSImageToHUDCamera();
};


class ROSPublisherInterface: public OpenThreads::Thread, public ROSInterface {
protected:
	int publish_rate;
	ros::Publisher pub_;
public:
	ROSPublisherInterface(std::string topic, int publish_rate);

	virtual void createPublisher(ros::NodeHandle &nh)=0;
	virtual void publish()=0;

	/* Thread code */
	void run();

	~ROSPublisherInterface();
};


class PATToROSOdom : public ROSPublisherInterface {
	osg::ref_ptr<osg::MatrixTransform> transform;
public:
	PATToROSOdom(osg::Group *rootNode,std::string vehicleName, std::string topic, int rate);

	void createPublisher(ros::NodeHandle &nh);

	void publish();
	
	~PATToROSOdom();
};


class ArmToROSJointState : public ROSPublisherInterface {
	boost::shared_ptr<URDFRobot> arm;
public:
	ArmToROSJointState(SimulatedIAUV *arm, std::string topic, int rate);

	void createPublisher(ros::NodeHandle &nh);

	void publish();
	
	~ArmToROSJointState();
};

	
class VirtualCameraToROSImage : public ROSPublisherInterface {
	VirtualCamera *cam;
        boost::shared_ptr<image_transport::ImageTransport> it;
	image_transport::Publisher img_pub_;
	std::string image_topic;
public:
	VirtualCameraToROSImage(VirtualCamera *camera, std::string topic, std::string info_topic, int rate);

	void createPublisher(ros::NodeHandle &nh);

	void publish();
	
	~VirtualCameraToROSImage();
};	

class RangeSensorToROSRange : public ROSPublisherInterface {
	VirtualRangeSensor *rs;
public:
	RangeSensorToROSRange(VirtualRangeSensor *rangesensor, std::string topic, int rate);

	void createPublisher(ros::NodeHandle &nh);

	void publish();
	
	~RangeSensorToROSRange();
};	
#endif
		
#endif /* ROSINTERFACE_H_ */

