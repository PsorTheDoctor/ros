#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Float32.h>

using namespace std;

float sum = 0;

void sinus_cb(const std_msgs::Float32::ConstPtr& msg) {  
	
	sum += msg->data;
	ROS_INFO("Received: %f Sum: %f", msg->data, sum);
}
	
int main(int argc, char* argv[]) {  

	ros::init(argc, argv, "my_subscriber");  
	ros::NodeHandle nh;  
	ros::Subscriber sub = nh.subscribe<std_msgs::Float32>("sinus", 10, sinus_cb);  
	ros::spin();  
	
	return 0;
}
