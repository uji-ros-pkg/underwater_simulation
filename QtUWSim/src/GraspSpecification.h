/*
 * GraspSpecification.h
 *
 *  Created on: 25/06/2012
 *      Author: toni
 */

#ifndef GRASPSPECIFICATION_H_
#define GRASPSPECIFICATION_H_

#include <ros/ros.h>
#include <visualization_msgs/InteractiveMarkerFeedback.h>
#include <ConfigXMLParser.h>
#include <OpenThreads/Thread>
#include "SceneBuilder.h"


class GraspSpecification: public OpenThreads::Thread{
private:
	ros::NodeHandle nh_;
	ros::Subscriber feedback_sub;
	visualization_msgs::InteractiveMarkerFeedback::ConstPtr lastFeedback;
	SceneBuilder *scene;
	std::vector<double> offsetp_;
	std::vector<double> offsetr_;

	void ReadFeedbackCallback(const visualization_msgs::InteractiveMarkerFeedback::ConstPtr& msg);
public:
	GraspSpecification();
	~GraspSpecification();
	void newPath(std::vector<double> offsetp, std::vector<double> offsetr, SceneBuilder *sceneBuilder);
	void run();
};


#endif /* GRASPSPECIFICATION_H_ */
