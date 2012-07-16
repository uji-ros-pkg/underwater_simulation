#ifndef HANDINTERACTIVEMARKER_H
#define HANDINTERACTIVEMARKER_H
#include <ros/ros.h>

#include <interactive_markers/interactive_marker_server.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>

#include <OpenThreads/Thread>

#include <math.h>



class HandInteractiveMarker: public OpenThreads::Thread {
	private:
		std::string package_, path_;
		
		
		
		
		void alignMarker( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );
		visualization_msgs::Marker makeBox( visualization_msgs::InteractiveMarker &msg );
		visualization_msgs::InteractiveMarkerControl& makeBoxControl( visualization_msgs::InteractiveMarker &msg );
		void saveMarker( visualization_msgs::InteractiveMarker int_marker );
		void make6DofMarker( bool fixed );
		void run();
	public:
		HandInteractiveMarker(std::string, std::string);
		~HandInteractiveMarker();
		
		//static void processFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );
		 static	void frameCallback(const ros::TimerEvent&);
	
};




#endif
