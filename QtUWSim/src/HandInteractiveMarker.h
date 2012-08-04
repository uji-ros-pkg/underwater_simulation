#ifndef HANDINTERACTIVEMARKER_H
#define HANDINTERACTIVEMARKER_H

#include <SceneBuilder.h>
#include <ConfigXMLParser.h>
#include <SimulatedIAUV.h>

#include <ros/ros.h>
#include <robot_state_publisher/joint_state_listener.h>
#include <interactive_markers/interactive_marker_server.h>
#include <im_joint_state_publisher/im_joint_state_publisher.h>


#include <OpenThreads/Thread>


/** Loads a URDF File and executes im_joint_state_publisher and robot_state_publisher so that the robot can be moved with interactive markers
 */
class HandInteractiveMarker: public OpenThreads::Thread {
	friend class MainWindow;

	private:
		boost::shared_ptr<SceneBuilder> scene_builder_;
		std::string name_;
		std::string fullpath_;
		
		boost::shared_ptr<robot_state_publisher::JointStateListener> tf_state_publisher;
		boost::shared_ptr<im_joint_state_publisher::ImJointStatePublisher> joint_state_publisher;

		boost::shared_ptr<SimulatedIAUV> uwsim_object; //The UWSim SimulatedIAUV object created from a URDF
		boost::shared_ptr<ROSJointStateToArm> ros_joint_states; //The ROS interface for updating joints

		bool stop_;

		boost::shared_ptr<interactive_markers::InteractiveMarkerServer> im_server;

		bool init();

		void processIMFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );

		void run();
	public:
		geometry_msgs::Pose pose;

		HandInteractiveMarker(boost::shared_ptr<SceneBuilder> scene_builder, std::string name, std::string package, std::string path);
		HandInteractiveMarker(boost::shared_ptr<SceneBuilder> scene_builder, std::string name, std::string fullpath);
		~HandInteractiveMarker();
};

#endif
