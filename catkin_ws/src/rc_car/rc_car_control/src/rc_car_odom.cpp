#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include "std_msgs/String.h"
#include <sstream>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <gazebo_msgs/ModelStates.h>

	
	std_msgs::String test;
	std_msgs::Float64 sen_xf;
	std_msgs::Float64 sen_zf;
	std::stringstream ss;
	float chassisWidth = 0.2;
 	float wheelWidth = 0.05;
	float wheelRadius = 0.1;
	float sen_x = 0;
	float sen_z = 0;
	bool unvaid=true;
	float i=0;


void sensor_ck(const sensor_msgs::JointState::ConstPtr& msg){
	sen_x = (msg->velocity[0] + msg->velocity[1])*wheelRadius/2;
	sen_z = (msg->velocity[1] - msg->velocity[0])*wheelRadius/(2*(chassisWidth+wheelWidth)/2);
}


int main(int argc, char** argv) {

	ros::init(argc, argv, "odom");
	ros::NodeHandle n;
	ros::Subscriber sensor_sub = n.subscribe("joint_states", 1000, sensor_ck);
	ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
	tf::TransformBroadcaster odom_broadcaster;
//	ros::Publisher test_pub = n.advertise<std_msgs::String>("sensor", 50);
	ros::Publisher  sen_x_pub = n.advertise<std_msgs::Float64>("sen_x", 50);
	ros::Publisher  sen_z_pub = n.advertise<std_msgs::Float64>("sen_z", 50);
//	ros::Subscriber  sen_sub = n.subscribe("/gazebo/model_states",1000, sensorG_ck);
	ros::Rate loop_rate(20);
	
	  double x = 0.0;
	  double y = 0.0;
	  double th = 0.0;
	
	  double vx = 0.0;
	  double vy = 0.0;
 	  double vth = 0.0;
	  double dt;

	ros::WallTime current_time;
	ros::WallTime last_time;
	ros::Time now_time;

	last_time = ros::WallTime::now();

	while (ros::ok()) {
		current_time = ros::WallTime::now();
		now_time = ros::Time::now();
		dt = (current_time-last_time).toSec();
		sen_xf.data=sen_x;
		sen_zf.data=sen_z;
	//test	
    	//	ss << cmd_z-sen_z<<";"<< sen_z<<";"<<cmd_x-sen_x<<";"<< sen_x<<'\n';
 	//	test.data = ss.str();
	//test
		vx = sen_x;
		vy = 0;
		vth = sen_z;

	    	//compute odometry in a typical way given the velocities of the robot
    		double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
    		double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
    		double delta_th = vth * dt;

    		x += delta_x;
    		y += delta_y;
    		th += delta_th;

    		//since all odometry is 6DOF we'll need a quaternion created from yaw
    		geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

    		//first, we'll publish the transform over tf
    		geometry_msgs::TransformStamped odom_trans;
    		odom_trans.header.stamp = now_time;
    		odom_trans.header.frame_id = "/odom";
    		odom_trans.child_frame_id = "/base_footprint";

    		odom_trans.transform.translation.x = x;
    		odom_trans.transform.translation.y = y;
   		odom_trans.transform.translation.z = 0.0;
    		odom_trans.transform.rotation = odom_quat;

    		//send the transform
    		odom_broadcaster.sendTransform(odom_trans);

    		nav_msgs::Odometry odom;
    		odom.header.stamp = now_time;
   		odom.header.frame_id = "/odom";

   		//set the position
   		odom.pose.pose.position.x = x;
   		odom.pose.pose.position.y = y;
   		odom.pose.pose.position.z = 0.0;
   		odom.pose.pose.orientation = odom_quat;

  		//set the velocity
		odom.child_frame_id = "/base_footprint";
		odom.twist.twist.linear.x = vx;
  		odom.twist.twist.linear.y = vy;
		odom.twist.twist.angular.z = vth;

		last_time = current_time;

		odom_pub.publish(odom);
//		test_pub.publish(test);
		sen_x_pub.publish(sen_xf);
		sen_z_pub.publish(sen_zf);

		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
