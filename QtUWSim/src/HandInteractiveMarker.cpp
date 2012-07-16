#include "HandInteractiveMarker.h"

	 boost::shared_ptr<interactive_markers::InteractiveMarkerServer> handInteractiveMarkerServer;



void HandInteractiveMarker::frameCallback(const ros::TimerEvent&)
{
	static uint32_t counter = 0;

	static tf::TransformBroadcaster br;

	tf::Transform t;

	ros::Time time = ros::Time::now();

	t.setOrigin(tf::Vector3(0.0, 0.0, sin(float(counter)/140.0) * 2.0));
	t.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));
	br.sendTransform(tf::StampedTransform(t, time, "base_link", "moving_frame"));

	t.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
	t.setRotation(tf::createQuaternionFromRPY(0.0, float(counter)/140.0, 0.0));
	br.sendTransform(tf::StampedTransform(t, time, "base_link", "rotating_frame"));

	++counter;
}

void handInteractiveMarkerProcessFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
	std::ostringstream s;
	s << "Feedback from marker '" << feedback->marker_name << "' "
			<< " / control '" << feedback->control_name << "'";

	std::ostringstream mouse_point_ss;
	if( feedback->mouse_point_valid )
	{
		mouse_point_ss << " at " << feedback->mouse_point.x
				<< ", " << feedback->mouse_point.y
				<< ", " << feedback->mouse_point.z
				<< " in frame " << feedback->header.frame_id;
	}

	switch ( feedback->event_type )
	{
	case visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK:
		ROS_INFO_STREAM( s.str() << ": button click" << mouse_point_ss.str() << "." );
		std::cout<<"button click-------------------------------"<<std::endl;
		break;

	case visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT:
		ROS_INFO_STREAM( s.str() << ": menu item " << feedback->menu_entry_id << " clicked" << mouse_point_ss.str() << "." );
		break;

	case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:
		ROS_INFO_STREAM( s.str() << ": pose changed"
				<< "\nposition = "
				<< feedback->pose.position.x
				<< ", " << feedback->pose.position.y
				<< ", " << feedback->pose.position.z
				<< "\norientation = "
				<< feedback->pose.orientation.w
				<< ", " << feedback->pose.orientation.x
				<< ", " << feedback->pose.orientation.y
				<< ", " << feedback->pose.orientation.z
				<< "\nframe: " << feedback->header.frame_id
				<< " time: " << feedback->header.stamp.sec << "sec, "
				<< feedback->header.stamp.nsec << " nsec" );
		std::cout<<"pose update-------------------------------"<<std::endl;
		break;

	case visualization_msgs::InteractiveMarkerFeedback::MOUSE_DOWN:
		ROS_INFO_STREAM( s.str() << ": mouse down" << mouse_point_ss.str() << "." );
		std::cout<<"mouse down-------------------------------"<<std::endl;
		break;

	case visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP:
		ROS_INFO_STREAM( s.str() << ": mouse up" << mouse_point_ss.str() << "." );
		std::cout<<"mouse up-------------------------------"<<std::endl;
		break;
	}

	handInteractiveMarkerServer->applyChanges();
}

HandInteractiveMarker::HandInteractiveMarker(std::string package, std::string path){
	package_=package;
	path_=path;
	startThread();
}

HandInteractiveMarker::~HandInteractiveMarker(){
	cancel();
	join();
	handInteractiveMarkerServer.reset();
}



void HandInteractiveMarker::run(){

	ros::NodeHandle n;
	ros::Timer frame_timer = n.createTimer(ros::Duration(0.01), frameCallback);

	handInteractiveMarkerServer.reset( new interactive_markers::InteractiveMarkerServer("basic_controls","",false) );

	ros::Duration(0.1).sleep();

	make6DofMarker( false );


	handInteractiveMarkerServer->applyChanges();

}



void HandInteractiveMarker::alignMarker( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
	geometry_msgs::Pose pose = feedback->pose;

	pose.position.x = round(pose.position.x-0.5)+0.5;
	pose.position.y = round(pose.position.y-0.5)+0.5;

	ROS_INFO_STREAM( feedback->marker_name << ":"
			<< " aligning position = "
			<< feedback->pose.position.x
			<< ", " << feedback->pose.position.y
			<< ", " << feedback->pose.position.z
			<< " to "
			<< pose.position.x
			<< ", " << pose.position.y
			<< ", " << pose.position.z );

	handInteractiveMarkerServer->setPose( feedback->marker_name, pose );
	handInteractiveMarkerServer->applyChanges();
}

visualization_msgs::Marker HandInteractiveMarker::makeBox( visualization_msgs::InteractiveMarker &msg )
{
	visualization_msgs::Marker marker;

	marker.type = visualization_msgs::Marker::MESH_RESOURCE;
	//marker.type = Marker::SPHERE;
	std::string str_aux="package://";
	str_aux+=package_;
	str_aux+="/";
	str_aux+=path_;
	marker.mesh_resource=str_aux;
	marker.mesh_use_embedded_materials=true;
	marker.scale.x = msg.scale * 1;
	marker.scale.y = msg.scale * 1;
	marker.scale.z = msg.scale * 1;


	marker.color.r = 0.5;
	marker.color.g = 0.5;
	marker.color.b = 0.5;
	marker.color.a = 1.0;

	return marker;
}

visualization_msgs::InteractiveMarkerControl& HandInteractiveMarker::makeBoxControl( visualization_msgs::InteractiveMarker &msg )
{
	visualization_msgs::InteractiveMarkerControl control;
	control.always_visible = true;
	control.markers.push_back( makeBox(msg) );
	msg.controls.push_back( control );

	return msg.controls.back();
}

void HandInteractiveMarker::saveMarker( visualization_msgs::InteractiveMarker int_marker )
{
	handInteractiveMarkerServer->insert(int_marker);

	handInteractiveMarkerServer->setCallback(int_marker.name, &handInteractiveMarkerProcessFeedback);
}

void HandInteractiveMarker::make6DofMarker( bool fixed )
{
	visualization_msgs::InteractiveMarker int_marker;
	int_marker.header.frame_id = "/base_link";

	int_marker.scale = 1;
	//int_marker.pose.position.y=-2;

	int_marker.name = "simple_6dof";
	int_marker.description = "";

	// insert a box
	makeBoxControl(int_marker);

	visualization_msgs::InteractiveMarkerControl control;

	if ( fixed )
	{
		int_marker.name += "_fixed";
		int_marker.description += "\n(fixed orientation)";
		control.orientation_mode = visualization_msgs::InteractiveMarkerControl::FIXED;
	}

	control.orientation.w = 1;
	control.orientation.x = 1;
	control.orientation.y = 0;
	control.orientation.z = 0;
	control.name = "rotate_x";
	control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
	int_marker.controls.push_back(control);
	control.name = "move_x";
	control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
	int_marker.controls.push_back(control);

	control.orientation.w = 1;
	control.orientation.x = 0;
	control.orientation.y = 1;
	control.orientation.z = 0;
	control.name = "rotate_z";
	control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
	int_marker.controls.push_back(control);
	control.name = "move_z";
	control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
	int_marker.controls.push_back(control);

	control.orientation.w = 1;
	control.orientation.x = 0;
	control.orientation.y = 0;
	control.orientation.z = 1;
	control.name = "rotate_y";
	control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
	int_marker.controls.push_back(control);
	control.name = "move_y";
	control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
	int_marker.controls.push_back(control);

	handInteractiveMarkerServer->insert(int_marker);
	
	handInteractiveMarkerServer->setCallback(int_marker.name, &handInteractiveMarkerProcessFeedback);
}

	
