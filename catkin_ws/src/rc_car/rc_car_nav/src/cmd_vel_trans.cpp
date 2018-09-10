#include <ros/ros.h>
#include "std_msgs/String.h"
#include <sstream>
#include <iostream>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <cmath>
#include <algorithm>
	
	geometry_msgs::Twist cmd_vel;
	float cmd_x = 0;
	float cmd_y = 0;
	float trans_x = 0;
	float trans_y = 0;
	double roll=0;
	double pitch=0;
	double yaw=0;

void cmd_vel_ck(const geometry_msgs::Twist::ConstPtr& msg){
	cmd_x = msg -> linear.x;
	cmd_y = msg -> linear.y;
}

void odom_ck(const nav_msgs::Odometry::ConstPtr& msg){
	tf::Quaternion q(msg->pose.pose.orientation.x,msg->pose.pose.orientation.y,msg->pose.pose.orientation.z,msg->pose.pose.orientation.w);
	tf::Matrix3x3 m(q);
	m.getRPY(roll,pitch,yaw);
}

int main(int argc, char** argv) {

	ros::init(argc, argv, "cmd_vel_trans");
	ros::NodeHandle n;
	ros::Subscriber cmd_sub = n.subscribe("/rc_car/cmd_vel_trans", 1000, cmd_vel_ck);
	ros::Subscriber odom_sub = n.subscribe("/rc_car/odom", 1000, odom_ck);

	ros::Publisher  cmd_pub = n.advertise<geometry_msgs::Twist>("/rc_car/cmd_vel", 50);
	ros::Rate loop_rate(20);

	while (ros::ok()) {
//		std::cout<<roll<<'\t'<<pitch<<'\t'<<yaw<<std::endl;
		trans_x = cos(yaw)*cmd_x+sin(yaw)*cmd_y;
		trans_y = -sin(yaw)*cmd_x+cos(yaw)*cmd_y;
		if (trans_x >= 0){
		   cmd_vel.linear.x = trans_x;
		}
		else{
		   cmd_vel.linear.x = 0; 
		}

		cmd_vel.linear.x = std::min(0.5,cmd_vel.linear.x);
		cmd_vel.angular.z = atan2(trans_y,trans_x)*5;
		cmd_vel.angular.z = std::min(1.0,cmd_vel.angular.z);
		cmd_vel.angular.z = std::max(-1.0,cmd_vel.angular.z);

		cmd_vel.linear.y = 0;
		cmd_vel.linear.z = 0;
		cmd_vel.angular.x = 0;
		cmd_vel.angular.y = 0;

		std::cout<<trans_x<<'\t'<<trans_y<<'\t'<<cmd_vel.linear.x<<'\t'<<cmd_vel.angular.z<<std::endl;

		cmd_pub.publish(cmd_vel);

		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
