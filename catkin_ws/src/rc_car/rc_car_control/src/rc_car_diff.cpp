#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include "std_msgs/String.h"
#include <sstream>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

	
	std_msgs::Float64 v_left;
	std_msgs::Float64 v_right;
	std_msgs::Float64 sen_xf;
	std_msgs::Float64 sen_zf;
	float chassisWidth = 0.2;
 	float wheelWidth = 0.05;
	float wheelRadius = 0.1;
	float Totalmass = 9.5;//5+2+2+0.5
	double dt = 1;//desire reach time
	float Iz= 0.6463;
	float Iw= 0.01;
	float cmd_x = 0;
	float cmd_z = 0;
	float sen_x = 0;
	float sen_z = 0;
	float r=(chassisWidth+wheelWidth)/2;


void cmd_vel_ck(const geometry_msgs::Twist::ConstPtr& msg){
	cmd_x = msg -> linear.x;
	cmd_z = msg -> angular.z;
}

void sen_x_ck(const std_msgs::Float64::ConstPtr& msg){
	sen_x = msg->data;
}
void sen_z_ck(const std_msgs::Float64::ConstPtr& msg){
	sen_z = msg->data;
}


int main(int argc, char** argv) {

	ros::init(argc, argv, "diff_driver");
	ros::NodeHandle n;
	ros::Subscriber cmd_sub = n.subscribe("cmd_vel", 1000, cmd_vel_ck);
	ros::Subscriber sen_x_sub = n.subscribe("sen_x", 1000, sen_x_ck);
	ros::Subscriber sen_z_sub = n.subscribe("sen_z", 1000, sen_z_ck);

	ros::Publisher  left_pub = n.advertise<std_msgs::Float64>("leftWheel_effort_controller/command", 50);
	ros::Publisher  right_pub = n.advertise<std_msgs::Float64>("rightWheel_effort_controller/command", 50);
	ros::Rate loop_rate(20);

	while (ros::ok()) {
		v_right.data = (Iw/wheelRadius+wheelRadius*Totalmass*r/(2*r))/dt*(cmd_x-sen_x) 
			     + (Iw*r/wheelRadius+Iz*wheelRadius/(2*r))/dt*(cmd_z-sen_z);
		v_left.data = (Iw/wheelRadius+wheelRadius*Totalmass*r/(2*r))/dt*(cmd_x-sen_x)
			     + (-Iw*r/wheelRadius-Iz*wheelRadius/(2*r))/dt*(cmd_z-sen_z);

 		left_pub.publish(v_left);
 		right_pub.publish(v_right);

		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
