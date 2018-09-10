#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>


	float x = 0.0; 
	float y = 0.0;
	float th = 0.0;
	
	geometry_msgs::TransformStamped odom_trans;

void odom_ck(const nav_msgs::Odometry::ConstPtr& msg){
    odom_trans.header.stamp = msg->header.stamp;
    odom_trans.transform.translation.x = msg->pose.pose.position.x;
    odom_trans.transform.translation.y = msg->pose.pose.position.y;
    odom_trans.transform.translation.z = msg->pose.pose.position.z;

    odom_trans.transform.rotation.x = msg->pose.pose.orientation.x;
    odom_trans.transform.rotation.y = msg->pose.pose.orientation.y;
    odom_trans.transform.rotation.z = msg->pose.pose.orientation.z;
    odom_trans.transform.rotation.w = msg->pose.pose.orientation.w;
}

int main(int argc, char** argv) {

	ros::init(argc, argv, "odom_subscriber");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("/odom", 1, odom_ck); //no odom publish

	tf::TransformBroadcaster broadcaster;
	ros::Rate loop_rate(20);


	while (ros::ok()) {


		odom_trans.header.frame_id = "/tingbot/odom";
		odom_trans.child_frame_id = "/tingbot/base_footprint";
		
		// publishing the odometry and the new tf
		broadcaster.sendTransform(odom_trans);

		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
