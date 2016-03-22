#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <math.h>

#define FRONT 0
#define RIGHT 1
#define LEFT 2

#define Kp 0.5
#define Kd 100
#define T 1/180

#define Kp_x 10
#define Kd_x 100

#define SENSING_DISTANCE 0.75
#define AVOID_ANGULAR_K 10
#define CRASHING_DISTANCE 0.3

ros::Publisher pub;
geometry_msgs::Twist robot_speeds;


int crashflag;
float sensorData[2][3];


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
    if(((sensorData[0][FRONT] <= CRASHING_DISTANCE) && sensorData[0][FRONT] != 0) || crashflag == 1){
        if(sensorData[0][FRONT] >= sensorData[0][LEFT]){
            robot_speeds.angular.z = 0.7;
            robot_speeds.linear.x = -1;
        }
        else{
            robot_speeds.angular.z = -0.7;
            robot_speeds.linear.x = -1;
        }
        if((sensorData[0][FRONT] >= (1- CRASHING_DISTANCE)) || sensorData[0][FRONT] == 0 ){
            crashflag = 0;
            pub.publish(robot_speeds);
            return;
        }
        ROS_INFO("ANDANDO DE RÉ");
        ROS_INFO("%f %f",robot_speeds.angular.z,robot_speeds.linear.x);
        pub.publish(robot_speeds);
        crashflag = 1;
        return;
    }
    if (sensorData[0][FRONT] >= SENSING_DISTANCE || sensorData[0][RIGHT] >= SENSING_DISTANCE || sensorData[0][LEFT] >= SENSING_DISTANCE ){
        if(sensorData[0][FRONT] != 0){
            robot_speeds.linear.x = 1.5;
            if(sensorData[0][LEFT] > sensorData[0][RIGHT]){
                //robot_speeds.angular.z = -(Kp*sensorData[0][LEFT] + Kd*(sensorData[0][LEFT] - sensorData[1][LEFT])/T);
                robot_speeds.angular.z = -0.5;
            }
            else{
               // robot_speeds.angular.z = Kp*sensorData[0][RIGHT] + Kd*(sensorData[0][RIGHT] - sensorData[1][RIGHT])/T;
                robot_speeds.angular.z = 0.5;
            }
        }
        else{
            int aux = biggest(sensorData[0]);
            if(aux == RIGHT){
               robot_speeds.angular.z = Kp*sensorData[0][RIGHT] + Kd*(sensorData[0][RIGHT] - sensorData[0][RIGHT])/T;
            }
            if(aux == LEFT){
                robot_speeds.angular.z = -(Kp*sensorData[0][LEFT] + Kd*(sensorData[0][LEFT] - sensorData[1][LEFT])/T);
            }
        }
    }
    //ROS_INFO("Setting robot speeds x = %f, theta = %f",robot_speeds.linear.x,robot_speeds.angular.z);
    pub.publish(robot_speeds);
}

void frontCallback(const std_msgs::Float32Ptr &msg){
    sensorData[1][FRONT] = sensorData[0][FRONT];
    sensorData[0][FRONT] = msg->data;
}

void rightCallback(const std_msgs::Float32Ptr &msg){
    sensorData[1][RIGHT] = sensorData[0][RIGHT];
    sensorData[0][RIGHT] = msg->data;
}

void leftCallback(const std_msgs::Float32Ptr &msg){
    sensorData[1][LEFT] = sensorData[0][LEFT];
    sensorData[0][LEFT] = msg->data;
}

void speedCallback(const geometry_msgs::TwistPtr &msg){
    robot_speeds = *msg;
}

int main(int argc, char *argv[]){
    ros::init(argc, argv, "obstacle");
    ros::NodeHandle node;
    ros::Subscriber frontSub = node.subscribe("vrep/fantasmao/frontSonar",1,frontCallback);
    ros::Subscriber rightSub = node.subscribe("vrep/fantasmao/rightSonar",1,rightCallback);
    ros::Subscriber leftSub = node.subscribe("vrep/fantasmao/leftSonar",1,leftCallback);

    ros::Subscriber speedSub = node.subscribe("fantasmao/desSpeed",1,speedCallback);
    
    pub = node.advertise<geometry_msgs::Twist>("cmd_vel",1);
    
    sensorData[0][FRONT] = 0;
    sensorData[0][RIGHT] = 0;
    sensorData[0][LEFT] = 0;
    crashflag = 0;
    
    ros::Rate loopRate(180);
    
    while(ros::ok){
        ros::spinOnce();
        avoid();
        loopRate.sleep();
    } 
    return 0;
}