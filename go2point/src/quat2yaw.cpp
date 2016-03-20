#include <iostream>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/tf.h>

void callback(const nav_msgs::OdometryPtr &msg){
    double yaw = tf::getYaw(msg->pose.pose.orientation);
    yaw = yaw*180/M_PI;
    ROS_INFO("Yaw: %f",yaw);
}

int main(int argc, char *argv[]){
    ros::init(argc, argv, "quat2yaw");
    ros::NodeHandle node;
    ros::Subscriber sub = node.subscribe("vrep/vehicle/odometry",1,callback);
    ros::spin();
}
