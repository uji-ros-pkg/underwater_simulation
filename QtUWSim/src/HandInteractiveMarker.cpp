#include "HandInteractiveMarker.h"

#include <ros/package.h>

#include <kdl_parser/kdl_parser.hpp>

HandInteractiveMarker::HandInteractiveMarker(boost::shared_ptr<SceneBuilder> scene_builder, std::string name, std::string package, std::string path): scene_builder_(scene_builder), name_(name), stop_(false) {
	fullpath_=ros::package::getPath(package)+"/"+path;
	if (init()) startThread();
}

HandInteractiveMarker::HandInteractiveMarker(boost::shared_ptr<SceneBuilder> scene_builder, std::string name, std::string fullpath): scene_builder_(scene_builder), name_(name), stop_(false) {
	fullpath_=fullpath;
	if (init()) startThread();
}

HandInteractiveMarker::~HandInteractiveMarker(){
	scene_builder_->scene->localizedWorld->removeChild(uwsim_object->baseTransform);
	stop_=true;
	join();
}

void HandInteractiveMarker::processIMFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback ) {
	pose=feedback->pose;
	tf::Quaternion q;
	tf::quaternionMsgToTF(feedback->pose.orientation, q);
	double roll, pitch, yaw;
	btMatrix3x3(q).getRPY(roll, pitch, yaw);
	uwsim_object->setVehiclePosition(feedback->pose.position.x, feedback->pose.position.y, feedback->pose.position.z, roll, pitch, yaw);
}

bool HandInteractiveMarker::init() {
	// gets the location of the robot description on the parameter server
	KDL::Tree tree;
	if (!kdl_parser::treeFromFile(fullpath_, tree)){
		ROS_ERROR("Failed to extract kdl tree from xml robot description");
		return false;
	}

	//Load the URDF model in UWSim
	Vehicle v;
	ConfigFile parser;
	parser.processURDFFile(fullpath_, v);
	parser.postprocessVehicle(v);
	v.name=name_;
	uwsim_object=boost::shared_ptr<SimulatedIAUV>(new SimulatedIAUV(scene_builder_.get(), v));
	uwsim_object->setVehiclePosition(0,0,0,0,0,0);
	pose.position.x=0;
	pose.position.y=0;
	pose.position.z=0;
	pose.orientation.x=0;
	pose.orientation.y=0;
	pose.orientation.z=0;
	pose.orientation.w=1;
	scene_builder_->scene->localizedWorld->addChild(uwsim_object->baseTransform);
	//Create ROS interface
	ros_joint_states=boost::shared_ptr<ROSJointStateToArm>(new ROSJointStateToArm("/joint_states", uwsim_object, true));

	//Create the joint_state_publisher and the robot_state_publisher for the new URDF model
	joint_state_publisher=boost::shared_ptr<im_joint_state_publisher::ImJointStatePublisher>(new im_joint_state_publisher::ImJointStatePublisher());
	joint_state_publisher->loadNewRobot(fullpath_);
	tf_state_publisher=boost::shared_ptr<robot_state_publisher::JointStateListener>(new robot_state_publisher::JointStateListener(tree));

	//Create an interactive marker server to manage the absolute position/attitude
	im_server=boost::shared_ptr<interactive_markers::InteractiveMarkerServer>(new interactive_markers::InteractiveMarkerServer("hand_pose_im", "HandInteractiveMarker"));

	// create an interactive marker for our server
	visualization_msgs::InteractiveMarker int_marker;
	int_marker.header.frame_id = "world";
	int_marker.name = "hand_pose_im";
	int_marker.description = "Hand Pose IM";

	//TODO: set scale automatically according to the size of the hand

	//Create the controls: translation and rotation on all the axis
	visualization_msgs::InteractiveMarkerControl move_control;
	move_control.name = "move_x";
	move_control.orientation.w = 1;
	move_control.orientation.x = 1;
	move_control.orientation.y = 0;
	move_control.orientation.z = 0;
	move_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
	int_marker.controls.push_back(move_control);
	move_control.name = "rotate_x";
	move_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
	int_marker.controls.push_back(move_control);

	move_control.name = "move_y";
	move_control.orientation.w = 1;
	move_control.orientation.x = 0;
	move_control.orientation.y = 1;
	move_control.orientation.z = 0;
	move_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
	int_marker.controls.push_back(move_control);
	move_control.name = "rotate_y";
	move_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
	int_marker.controls.push_back(move_control);

	move_control.name = "move_z";
	move_control.orientation.w = 1;
	move_control.orientation.x = 0;
	move_control.orientation.y = 0;
	move_control.orientation.z = 1;
	move_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
	int_marker.controls.push_back(move_control);
	move_control.name = "rotate_z";
	move_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
	int_marker.controls.push_back(move_control);

	// add the interactive marker to our collection &
	// tell the server to call processFeedback() when feedback arrives for it
	im_server->insert(int_marker, boost::bind( &HandInteractiveMarker::processIMFeedback, this, _1));

	// 'commit' changes and send to all clients
	im_server->applyChanges();

	return true;
}

void HandInteractiveMarker::run() {
	ros::Rate r(50);
	while (!stop_ && ros::ok()) {
		ros::spinOnce();
		r.sleep();
	}
}
