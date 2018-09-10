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
	float angular_r=0;
	float angular_l=0;
	float K[2][2]={{0.3536,0.3536},{0.3536,-0.3536}};//feedback gain=[0.25 0.25;2 -2]
//	float N[2][2]={{7.0711,0},{0,0.8839}};//rescale N=[5 0;0 5]

//	float K[2][2]={{1,1},{1,1}};//feedback gain=[0.25 0.25;2 -2]
	float N[2][2]={{7.0711,0},{0,1.1}};//rescale N=[5 0;0 5]
	float r=(chassisWidth+wheelWidth)/2;


void cmd_vel_ck(const geometry_msgs::Twist::ConstPtr& msg){
	cmd_x = msg -> linear.x*N[0][0];
	cmd_z = msg -> angular.z*N[1][1];
}

void odom_ck(const nav_msgs::Odometry::ConstPtr& msg){
	sen_x = msg -> twist.twist.linear.x;
	sen_z = msg -> twist.twist.angular.z;
}

void sensor_ck(const sensor_msgs::JointState::ConstPtr& msg){
	angular_r = msg->velocity[1];
	angular_l = msg->velocity[0];
}

int main(int argc, char** argv) {

	ros::init(argc, argv, "diff_driver");
	ros::NodeHandle n;
	ros::Subscriber cmd_sub = n.subscribe("cmd_vel", 1000, cmd_vel_ck);
	ros::Subscriber sensor_sub = n.subscribe("joint_states", 1000, sensor_ck);
	ros::Subscriber odom_sub = n.subscribe("odom", 1000, odom_ck);

	ros::Publisher  left_pub = n.advertise<std_msgs::Float64>("leftWheel_effort_controller/command", 50);
	ros::Publisher  right_pub = n.advertise<std_msgs::Float64>("rightWheel_effort_controller/command", 50);
	ros::Rate loop_rate(20);

	while (ros::ok()) {
		v_right.data = (Iw/wheelRadius+wheelRadius*Totalmass*r/(2*r))/dt*(cmd_x-(angular_r*K[0][0]+angular_l*K[0][1]))+ (Iw*r/wheelRadius+Iz*wheelRadius/(2*r))/dt*(cmd_z-(angular_r*K[1][0]+angular_l*K[1][1]));
		v_left.data = (Iw/wheelRadius+wheelRadius*Totalmass*r/(2*r))/dt*(cmd_x-(angular_r*K[0][0]+angular_l*K[0][1])) + (-Iw*r/wheelRadius-Iz*wheelRadius/(2*r))/dt*(cmd_z-(angular_r*K[1][0]+angular_l*K[1][1]));
		
 		left_pub.publish(v_left);
 		right_pub.publish(v_right);

		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
