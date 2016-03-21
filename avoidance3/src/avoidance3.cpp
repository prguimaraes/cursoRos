#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <math.h>

#define FRONT 0
#define RIGHT 1
#define LEFT 2

ros::Publisher pub;
geometry_msgs::Twist robot_speeds;

float sonarData[3];

int biggest(float *v){
    if(v[RIGHT] >= v[FRONT] && v[RIGHT] >= v[LEFT]){
        return RIGHT;
    }
    if(v[LEFT] >= v[FRONT] && v[LEFT] >= v[FRONT]){
        return LEFT;
    }
    if(v[FRONT] >= v[RIGHT] && v[FRONT] >= v[LEFT]){
        return FRONT;
    }
}

void callbackFront(const std_msgs::Float32Ptr &msg){
    sonarData[FRONT] = msg->data;
}

void callbackRight(const std_msgs::Float32Ptr &msg){
    sonarData[RIGHT] = msg->data;
}

void callbackLeft(const std_msgs::Float32Ptr &msg){
    sonarData[LEFT] = msg->data;
}

int main(int argc, char *argv[]){
    ros::init(argc, argv, "obstacle_avoidance");
    ros::NodeHandle node;
    ros::Subscriber frontsub = node.subscribe("vrep/vehicle/frontSonar",1,callbackFront);
    ros::Subscriber rightsub = node.subscribe("vrep/vehicle/rightSonar",1,callbackRight);
    ros::Subscriber leftsub = node.subscribe("vrep/vehicle/leftSonar",1,callbackLeft);
    sonarData[FRONT] = 0;
    sonarData[RIGHT] = 0;
    sonarData[LEFT] = 0;
    ros::Rate loopRate(180);
    
    pub = node.advertise<geometry_msgs::Twist>("cmd_vel",1);
    
    while(ros::ok){
        ros::spinOnce(); 
        loopRate.sleep();
    }
}