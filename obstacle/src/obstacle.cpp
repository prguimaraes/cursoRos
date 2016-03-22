#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <math.h>

#define FRONT 0
#define RIGHT 1
#define LEFT 2

#define SENSING_DISTANCE 0
#define AVOID_ANGULAR_SPEED 3

ros::Publisher pub;
geometry_msgs::Twist robot_speeds;



float sensorData[3];


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

void avoid(){
    if (sensorData[FRONT] != SENSING_DISTANCE || sensorData[RIGHT] != SENSING_DISTANCE || sensorData[LEFT] != SENSING_DISTANCE ){
        int aux = biggest(sensorData);
        if(aux == FRONT){
            
        }
        if(aux == RIGHT){
            robot_speeds.angular.z = AVOID_ANGULAR_SPEED;
        }
        if(aux == LEFT){
            robot_speeds.angular.z = -AVOID_ANGULAR_SPEED;
        }
    }
    ROS_INFO("Setting robot speeds x = %f, theta = %f",robot_speeds.linear.x,robot_speeds.angular.z);
    pub.publish(robot_speeds);
}

void frontCallback(const std_msgs::Float32Ptr &msg){
    sensorData[FRONT] = msg->data;
}

void rightCallback(const std_msgs::Float32Ptr &msg){
    sensorData[RIGHT] = msg->data;
}

void leftCallback(const std_msgs::Float32Ptr &msg){
    sensorData[LEFT] = msg->data;
}

void speedCallback(const geometry_msgs::TwistPtr &msg){
    robot_speeds = *msg;
}

int main(int argc, char *argv[]){
    ros::init(argc, argv, "obstacle");
    ros::NodeHandle node;
    ros::Subscriber frontSub = node.subscribe("vrep/vehicle/frontSonar",1,frontCallback);
    ros::Subscriber rightSub = node.subscribe("vrep/vehicle/rightSonar",1,rightCallback);
    ros::Subscriber leftSub = node.subscribe("vrep/vehicle/leftSonar",1,leftCallback);

    ros::Subscriber speedSub = node.subscribe("fantasmao/desSpeed",1,speedCallback);
    
    pub = node.advertise<geometry_msgs::Twist>("cmd_vel",1);
    
    ros::Rate loopRate(180);
    
    while(ros::ok){
        ros::spinOnce();
        avoid();
        loopRate.sleep();
    } 
}