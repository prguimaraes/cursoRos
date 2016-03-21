#include <iostream>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <math.h>

#define K1 2
#define K2 3



ros::Publisher pub;
geometry_msgs::Twist robot_speeds;

double xf = 4, yf = 1.5;

double normalizeAngles(double angle){
    if(angle < 0){
        angle = angle - 360;
    }
    return angle;
}

void callback(const nav_msgs::OdometryConstPtr &msg){
    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;
    double yaw = tf::getYaw(msg->pose.pose.orientation);
    double theta = atan2(yf - y,xf - x);
    double d = sqrt(pow(xf-x,2) + pow(yf-y,2));
    
    if(d > 0.1 && d <= 0.5){
        robot_speeds.linear.x = K2*d;
        robot_speeds.angular.z = K1*(theta - yaw);
    }
    if(d > 0.5){
        robot_speeds.linear.x = 1.5;
        robot_speeds.angular.z = K1*(theta - yaw);

    }
    else{
        robot_speeds.linear.x = 0;
        robot_speeds.angular.z = 0;
    }
    ROS_INFO("d = %f, x = %f, y = %f",d,x,y);
    pub.publish(robot_speeds);
}

int main(int argc, char *argv[]){
    ros::init(argc, argv, "go2point");
    ros::NodeHandle node;
    ros::Subscriber sub = node.subscribe("vrep/vehicle/odometry",1,callback);

    
    pub = node.advertise<geometry_msgs::Twist>("cmd_vel",1);
    
    ros::spin();    
}