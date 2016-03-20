#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <math.h>

ros::Publisher pub;
geometry_msgs::Twist robot_speeds;

void callback(const std_msgs::Float32Ptr &msg){
    if(msg->data != 0){
        robot_speeds.linear.x = 0;
        robot_speeds.angular.z = 1;
    }
    else{
        robot_speeds.linear.x = 0.5;
        robot_speeds.angular.z = 0;
    }
    ROS_INFO("Setting robot speeds: x = %f, theta = %f",robot_speeds.linear.x,robot_speeds.angular.z);
    pub.publish(robot_speeds);
}

int main(int argc, char *argv[]){
    ros::init(argc, argv, "obstacle_avoidance");
    ros::NodeHandle node;
    ros::Subscriber sub = node.subscribe("vrep/vehicle/frontSonar",1,callback);
    
    pub = node.advertise<geometry_msgs::Twist>("robotSpeeds",1);
    
    ros::spin();    
}