#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <stdlib.h>

int main(int argc, char **argv) {
	if (argc != 7) {
                std::cerr << "Usage: " << argv[0] << "<topic> <q1> <q2> <q3> <q4> <q5>" << std::endl;
		std::cerr << "Units are radians" << std::endl;
                exit(0);
        }
	std::string topic(argv[1]);

	ros::init(argc, argv, "setJointPosition");
	ros::NodeHandle nh;
	ros::Publisher position_pub;
	position_pub=nh.advertise<sensor_msgs::JointState>(topic,1);
	ros::Rate rate(30);

	double q[5];
	for (int i=0; i<5; i++) q[i]=atof(argv[i+2]);

	while (ros::ok()) {
		
		sensor_msgs::JointState js;
        	js.name.push_back(std::string("q1"));
        	js.position.push_back(q[0]);
        	js.name.push_back(std::string("q2"));
        	js.position.push_back(q[1]);
        	js.name.push_back(std::string("q3"));
        	js.position.push_back(q[2]);
        	js.name.push_back(std::string("q4"));
        	js.position.push_back(q[3]);
        	js.name.push_back(std::string("q5"));
        	js.position.push_back(q[4]);

        	position_pub.publish(js);

		ros::spinOnce();
		rate.sleep();
	}

	return 0;
}
