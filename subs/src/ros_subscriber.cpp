#include <iostream>

#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>

int counter = 0;
std_msgs::Int32 msg_counter;
ros::Publisher pub;


void callback(const std_msgs::StringConstPtr &msg){
    counter++;
    ROS_INFO("Received: %s",msg->data.c_str());
    msg_counter.data = counter;
    pub.publish(msg_counter);
}

int main(int argc, char *argv[]){
    ros::init(argc, argv, "ros_sub");
    ros::NodeHandle node;
    ros::Subscriber sub = node.subscribe("ros_world",1,callback);
    pub = node.advertise<std_msgs::Int32>("ros_world/counter",1);
    ros::spin();
}
