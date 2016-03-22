#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>

#define b 0.35
#define r 0.1

#define MAX_ANGULAR_SPEED 15

ros::Publisher leftSpeed;
ros::Publisher rightSpeed;
std_msgs::Float32 leftMsg;
std_msgs::Float32 rightMsg;

float calculate_rightSpeed(float v, float w){
    return (v + b*w/2)/r;
}


float calculate_leftSpeed(float v, float w){
    return (v - b*w/2)/r;
}

void callback(const geometry_msgs::TwistPtr &msg){
    float wr, wl, rate;
    wr = calculate_rightSpeed(msg->linear.x,msg->angular.z);
    wl = calculate_leftSpeed(msg->linear.x,msg->angular.z);
    
    
    if(wl > MAX_ANGULAR_SPEED || wr > MAX_ANGULAR_SPEED){
        if(wl>wr){
            rate = wr/wl;
            wl = MAX_ANGULAR_SPEED;
            wr = rate*wl;
        }
        if(wr>wl){
            rate = wl/wr;
            wr = MAX_ANGULAR_SPEED;
            wl = rate*wr;
        }
    }
    
    
    
    rightMsg.data = wr;
    leftMsg.data = wl;
    ROS_INFO("Setting wheel angular speeds: R = %f L = %f",rightMsg.data,leftMsg.data);
    leftSpeed.publish(leftMsg);
    rightSpeed.publish(rightMsg);
}


int main(int argc, char *argv[]){
    ros::init(argc, argv, "kine_controller");
    ros::NodeHandle node;
    leftSpeed = node.advertise<std_msgs::Float32>("vrep/fantasmao/motorLeftSpeed",1);//("vrep/fantasmao/motorLeftSpeed",1);
    rightSpeed = node.advertise<std_msgs::Float32>("vrep/fantasmao/motorRightSpeed",1);
    
    ros::Subscriber robotSpeeds = node.subscribe("cmd_vel",1,callback);

    ros::spin();
}