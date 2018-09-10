#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include "std_msgs/String.h"
#include <sstream>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <gazebo_msgs/ModelStates.h>

	float i=0;

	geometry_msgs::TransformStamped odom_trans;
	nav_msgs::Odometry odom;

void sensorG_ck(const gazebo_msgs::ModelStates::ConstPtr& msg){
	 while (msg->name[i] != "tingbot"){
	   i++;
	}
	odom_trans.transform.translation.x = msg->pose[i].position.x;
	odom_trans.transform.translation.y = msg->pose[i].position.y;
	odom_trans.transform.translation.z = msg->pose[i].position.z;
	odom_trans.transform.rotation = msg->pose[i].orientation;
	odom.pose.pose = msg->pose[i];
	odom.twist.twist = msg->twist[i];
}

int main(int argc, char** argv) {

	ros::init(argc, argv, "odom");
	ros::NodeHandle n;
	ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
	tf::TransformBroadcaster odom_broadcaster;
	ros::Subscriber  sen_sub = n.subscribe("/gazebo/model_states",1000, sensorG_ck);
	ros::Rate loop_rate(20);
	  double dt;

	ros::WallTime current_time;
	ros::WallTime last_time;
	ros::Time now_time;

	last_time = ros::WallTime::now();

	odom.pose.pose.position.x = 0;
	odom.pose.pose.position.y = 0;
	odom.pose.pose.position.z = 0;
	odom.pose.pose.orientation.x = 0;
	odom.pose.pose.orientation.y = 0;
	odom.pose.pose.orientation.z = 0;
	odom.pose.pose.orientation.w = 0;
	odom.twist.twist.linear.x = 0;
	odom.twist.twist.linear.y = 0;
	odom.twist.twist.linear.z = 0;
	odom.twist.twist.angular.x = 0;
	odom.twist.twist.angular.y = 0;
	odom.twist.twist.angular.z = 0;

	while (ros::ok()) {
		current_time = ros::WallTime::now();
		now_time = ros::Time::now();
		dt = (current_time-last_time).toSec();

    		odom_trans.header.stamp = now_time;
    		odom_trans.header.frame_id = "/odom";
    		odom_trans.child_frame_id = "base_footprint";

		odom_broadcaster.sendTransform(odom_trans);

    		odom.header.stamp = now_time;
   		odom.header.frame_id = "/odom";

		odom.child_frame_id = "base_footprint";

		last_time = current_time;

		odom_pub.publish(odom);

		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
