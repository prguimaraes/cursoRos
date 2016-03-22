#include<ros/ros.h>
#include<nav_msgs/Odometry.h>
#include<geometry_msgs/Point.h>

#define CHANGEPATH_DISTANCE 0.7

//publisher and subscriber
ros::Publisher w1_pub;
ros::Publisher w2_pub;
ros::Subscriber car_position_sub;

geometry_msgs::Point w1, w2;

float px[19] = {10.400,11.250,12.425,14.625,14.525,9.000,7.750,-5.125,-6.425,-13.550,-13.725,-12.750,-2.125,-1.200,-2.150,-12.050,-12.900,-12.900,-13.100,-10.275};
float py[19] ={-0.225,-9.575,-10.650,-14.350,-15.500,-19.425,-20.175,-19.975,-19.500,-14.475,-13.475,-13.275,-12.350,-11.750,-11.125,-10.425,-10.050,-9.250,-1.975,0.075};

int i;

double calculateDistance(geometry_msgs::Point p1, geometry_msgs::Point p2){
    return sqrt(pow(p1.x - p2.x,2) + pow(p1.y - p2.y,2));
}

void change_path()
{
    i++;
    if (i ==19){
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