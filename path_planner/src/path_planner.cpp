#include<ros/ros.h>
#include<std_msgs/Float32MultiArray.h>
#include<nav_msgs/Odometry.h>
#include<geometry_msgs/Pose2D.h>
#include<math/math.h>

//publisher and subscriber
ros::Publisher array_pub;
ros::Subscriber car_position_sub;

geometry_msgs::Pose2D w1, w2;
std_msgs::Float32MultiArray path;
float px[10] = {10.700,10.700,15.250,8.550,-5.925,-14.175,-2.250,-0.875,-13.025,-12.950};
float py[10] ={0.150,-8.275,-15.250,-19.875,-19.950,-13.250,-12.525,-10.875,-10.275,-0.225};

int i =1;

change_path()
{
    i++;
    if (i ==10){
        i =0;
    }
    w1 = w2;
    w2.x = px[i];
    w2.y = py[i];
    
}

void odomCallBack(const nav_msgs::OdometryConstPtr &msg)
{
    
    //Determinar posicao atual
    geometry_msgs::Pose2D p;
    p.x = msg->pose.pose.position.x;
    p.y = msg->pose.pose.position.y;
    if ((p.y == w2.y) && (p.x == w2.x))
    {
        change_path();
    }
    path.data[0][0] = w1.x;
    path.data[0][1] = w1.y;
    path.data[1][0] = w2.x;
    path.data[1][1] = w2.y;
    array_pub.publish(path);
}


int main (argc, char ** argv)
{
    ros::init(argc,argv, "path_planner");
    ros::NodeHandle node;
    car_position_sub = node.subscribe<nav_msgs::Odometry>("vrep/fantasmao/odometry", 1, odomCallBack);
    path.layout.dim.size = 2;
    path.layout.dim.label = "position";
    path.layout.dim.stride = 2;
    array_pub = node.avertise<std_msgs::Float32MultiArray>("publishing_position", 1);
    w1.x = px[0];
    w1.y = py[0];
    w2.x = px[1];
    w2.y = py[1];
    ros::spin();
    
    
}