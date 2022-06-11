#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <math.h>

using namespace std;

int main(int argc, char* argv[]) {
	
	ros::init(argc, argv, "my_publisher");  
	ros::NodeHandle nh;  
	ros::Publisher pub = nh.advertise<std_msgs::Float32>("sinus", 10);  
	ros::Rate rate(10);
	
	while (ros::ok()) {
		for (int i = 0; i < 100; i++) {
			
			std_msgs::Float32 msg;    
			msg.data = sin(0.01 * i);
			   
			ROS_INFO("Sending: %f", msg.data);    
			pub.publish(msg);   
			ros::spinOnce();    
			rate.sleep();  
		}
	}  
	return 0;
}
