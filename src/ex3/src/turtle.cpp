#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Pose.h>
#include <math.h>

using namespace std;

float xTurtle = 0;
float yTurtle = 0;
float xTarget = 5.0;
float yTarget = 5.0;
float angleTurtle = 0;
float angleDiff = 0;
float angleFinal = 0;

geometry_msgs::Twist twist;

void pose(const turtlesim::Pose::ConstPtr& pos) {
	xTurtle = pos->x;
	yTurtle = pos->y;
	angleTurtle = pos->theta;
}

void target_cb(const turtlesim::Pose::ConstPtr& msg) {
	xTarget = msg->x;
	yTarget = msg->y;
}

int main(int argc, char* argv[]) {
	
	ros::init(argc, argv, "turtle");
	ros::NodeHandle nh;
	ros::Subscriber sub = nh.subscribe<turtlesim::Pose>("/turtle1/pose", 10, pose);
	ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);
	ros::Rate rate(50);
	
	while (ros::ok()) {
		geometry_msgs::Twist twist;
		angleDiff = atan2(yTarget - yTurtle, xTarget - xTurtle);
		angleFinal = angleDiff - angleTurtle;
		
		if(xTurtle < (xTarget - 0.5) || xTurtle > (xTarget + 0.5)) {
			
			if (abs(angleFinal) > 0.1) {
				twist.linear.x = 0.0;
				twist.angular.z = angleDiff * 2;
			} else {
				twist.angular.z = 0.0;
				twist.linear.x = 2.0;	
			}
		} else if(yTurtle < (yTarget - 0.5) || yTurtle > (yTarget + 0.5)) {
			
			if (abs(angleFinal) > 0.1) {
				twist.linear.x = 0.0;
				twist.angular.z = angleDiff * 2;
			} else {
				twist.angular.z = 0.0;
				twist.linear.x = 2.0;	
			}
		} else {
			twist.angular.z = 0.0;
			twist.linear.x = 0.0;
		}
		pub.publish(twist);
		ros::spinOnce();
		rate.sleep();
	}
	return 0;
}
