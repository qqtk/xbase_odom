// get ticks 'msg', publish /odom topic '
// and publish 'the odom/ base_link TransformStamped'msg to tf.
//
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>

#include "robbase_msg/encoders.h"
#include <string>

double left_ticks, right_ticks;
double left_ticks_prev, right_ticks_prev;
double delta_left_ticks, delta_right_ticks;
ros::Time current_time, last_time;

double self_x=0;
double self_y=0;
double self_th=0;
double base_width, ticks_per_meter;
ros::NodeHandle *private_n;

void ticksLR_callback(const robbase_msg::encoders& ticks_msg){
//    left_ticks = ticks_msg.lwheelticks;
    left_ticks = ticks_msg.ticks_l;
    right_ticks = ticks_msg.ticks_r;
} 
//
int main( int argc, char* argv[] ){

    ros::init(argc, argv, "base_odom_node" );
    ros::NodeHandle nh;
    private_n= new ros::NodeHandle("~");
    
    ros::Subscriber ticks_sub = nh.subscribe("/encoder", 20, ticksLR_callback);
    ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("/odom", 20);
    
    // nh.param("base_width", base_width, 0.5);
    double base_width;
    if(!private_n->getParam("base_width", base_width))
    {
        ROS_WARN("No base_width provided - default: 0.496m");
        base_width = 0.496;
    }

    int ticks_per_meter;
    // nh.param("ticks_per_meter", ticks_per_meter,88000);
    if(!private_n->getParam("ticks_per_meter", ticks_per_meter))
    {
        ROS_WARN("No ticks_per_meter provided - default: 24298");
        ticks_per_meter = 24298;
    }
    
    geometry_msgs::TransformStamped odom_transform_msg;
    tf::TransformBroadcaster odom_tf_broadcaster;
    ros::Rate loop_rate(20);
    ROS_INFO("Node base_odometry started");
 
    last_time = ros::Time::now();

    while (ros::ok()) {
        double dx, dr, dist, dtheta, d_left, d_right;
	double x, y;
	current_time = ros::Time::now(); 
	delta_left_ticks = left_ticks - left_ticks_prev;
	delta_right_ticks = right_ticks - right_ticks_prev;

	double elapsed_dt = (current_time - last_time).toSec();
        // ********************
        // * compute odometry *
        // ********************
        d_left = delta_left_ticks/ticks_per_meter;
        d_right= delta_right_ticks/ticks_per_meter;
        //distance traveled as average of both wheels
        dist = (d_left + d_right)/2;
        dtheta = (d_right - d_left)/ base_width;
        //calculate velocities
        dx = dist / elapsed_dt;
        dr = dtheta / elapsed_dt;

        //calculate distance traveled and final position
        if (dist != 0) {
            //calculate distance traveled
            x = cos(dtheta) * dist;
            y = -sin(dtheta) * dist;
            //calculate final position
            self_x = self_x + (cos(self_th) * x - sin(self_th) * y);
            self_y = self_y + (sin(self_th) * x + cos(self_th) * y);
        }
        if (dtheta != 0) {
            self_th = self_th + dtheta;
        }
	
	// We use a quaternion created from yaw
	geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(self_th);	
	// odom_quat = tf::createQuaternionMsgFromRollPitchYaw(0,0,self_th);

	// publish TransformStamped message for odom/base_link 'to topic /tf
	odom_transform_msg.header.frame_id = "odom";
	odom_transform_msg.child_frame_id = "base_link";
	odom_transform_msg.header.stamp = current_time; 
	odom_transform_msg.transform.translation.x = self_x; 
	odom_transform_msg.transform.translation.y = self_y; 
	odom_transform_msg.transform.translation.z = 0.0;
	odom_transform_msg.transform.rotation = odom_quat;

        // publish the /odom topic
        nav_msgs::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = "odom";
        //set the position
        odom.pose.pose.position.x = self_x;
        odom.pose.pose.position.y = self_y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = odom_quat;
        //set the velocity
        odom.child_frame_id = "base_link";
        odom.twist.twist.linear.x = dx;
        odom.twist.twist.linear.y = 0;
        odom.twist.twist.angular.z = dr;
        odom_pub.publish(odom);
	// publishing the odometry and the tf: odom/ base_link
	odom_tf_broadcaster.sendTransform(odom_transform_msg);

        last_time = current_time;
	right_ticks_prev = right_ticks;
	left_ticks_prev = left_ticks;

        loop_rate.sleep();
    }// end.while _ ros::ok '
}// end.main

