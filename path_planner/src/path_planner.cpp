#include<ros/ros.h>
#include<nav_msgs/Odometry.h>
#include<geometry_msgs/Point.h>

#define CHANGEPATH_DISTANCE 1

//publisher and subscriber
ros::Publisher w1_pub;
ros::Publisher w2_pub;
ros::Subscriber car_position_sub;

geometry_msgs::Point w1, w2;

float px[11] = {10.700,10.700,15.250,8.550,-5.925,-14.375,-14.375,-2.675,-0.875,-13.025,-12.950};
float py[11] ={0.150,-8.275,-15.250,-19.875,-19.950,-14.075,-13.250,-12.525,-10.875,-10.275,-0.225};

int i;

double calculateDistance(geometry_msgs::Point p1, geometry_msgs::Point p2){
    return sqrt(pow(p1.x - p2.x,2) + pow(p1.y - p2.y,2));
}

void change_path()
{
    i++;
    if (i ==11){
        i =0;
    }
    w1 = w2;
    w2.x = px[i];
    w2.y = py[i];
    
}

void odomCallBack(const nav_msgs::OdometryConstPtr &msg)
{
    
    //Determinar posicao atual
    geometry_msgs::Point p;
    p.x = msg->pose.pose.position.x;
    p.y = msg->pose.pose.position.y;
    double d = calculateDistance(p,w2);
    if (d < CHANGEPATH_DISTANCE)
    {
//        ROS_INFO("mudei");
        change_path();
    }
//    ROS_INFO("%f",d);
    ROS_INFO("w1 = (%f,%f) w2 = (%f,%f)",w1.x,w1.y,w2.x,w2.y);
    w1_pub.publish(w1);
    w2_pub.publish(w2);
}


int main (int argc, char **argv)
{
    ros::init(argc,argv, "path_planner");
    ros::NodeHandle node;
    car_position_sub = node.subscribe<nav_msgs::Odometry>("vrep/fantasmao/odometry", 1, odomCallBack);
    w1_pub = node.advertise<geometry_msgs::Point>("fantasmao/w1", 1);
    w2_pub = node.advertise<geometry_msgs::Point>("fantasmao/w2", 1);
    i = 1;
//    w1.x = 4;
//    w1.y = 1.5;
//    w2.x = -3;
//    w2.y = 1.5;
    w1.x = px[0];
    w1.y = py[0];
    w2.x = px[1];
    w2.y = py[1];
    ros::spin();    
}