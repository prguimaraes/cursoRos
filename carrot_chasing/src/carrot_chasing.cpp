#include <iostream>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <math.h>
#include <geometry_msgs/Point.h>

#define T 1/180

#define Kp 4
#define Kd 200 
#define Ki 0
#define K2 2


ros::Publisher pub;
geometry_msgs::Twist robot_speeds;

geometry_msgs::Point w1, w2, p;

double d_theta[2];
double d_theta_i;
double psi;

double delta = 0.5;

double normalizeAngles(double angle){
    if(angle > M_PI){
        angle = angle - 2*M_PI;
    }
    if (angle < -M_PI){
        angle = angle + 2*M_PI;
    }
    return angle;
}

double calculateDistance(geometry_msgs::Point p1, geometry_msgs::Point p2){
    return sqrt(pow(p1.x - p2.x,2) + pow(p1.y - p2.y,2));
}

void w1CallBack (const geometry_msgs::PointConstPtr &msg){
    w1 = *msg;
}

void w2CallBack (const geometry_msgs::PointConstPtr &msg){
    w2 = *msg;
}

void odomCallBack(const nav_msgs::OdometryConstPtr &msg){
    p.x = msg->pose.pose.position.x;
    p.y = msg->pose.pose.position.y;    
    psi = tf::getYaw(msg->pose.pose.orientation);
}

void calculateSpeeds(){
    
    double Ru = calculateDistance(w1,p);
    double Theta_u = atan2(p.y - w1.y, p.x - w1.x);
    double Theta = atan2(w2.y - w1.y, w2.x - w1.x);
    double Beta = Theta - Theta_u;
    double R = sqrt(pow(Ru,2) - pow(Ru*sin(Beta),2));
    geometry_msgs::Point c;
    
    c.x = (R + delta)*cos(Theta) + w1.x;
    c.y = (R + delta)*sin(Theta) + w1.y;
    
    double dist = calculateDistance(w2,p);
    
    double psi_d = atan2(c.y - p.y,c.x - p.x);
    
    d_theta[1] = d_theta[0];
    d_theta[0] = normalizeAngles(psi_d - psi);
    d_theta_i = d_theta_i + d_theta[0];
    
    robot_speeds.linear.x = 1.5;
    robot_speeds.angular.z = Kp*d_theta[0] + Kd*(d_theta[0] - d_theta[1])/T + Ki*d_theta_i;
    

    ROS_INFO("d = %f, x = %f, y = %f",dist,p.x,p.y);
    pub.publish(robot_speeds);
}

int main(int argc, char *argv[]){
    ros::init(argc, argv, "fantasmao_carrot");
    ros::NodeHandle node;
    ros::Subscriber odom_sub = node.subscribe("vrep/fantasmao/odometry",1,odomCallBack);
    ros::Subscriber w1_sub = node.subscribe("fantasmao/w1",1,w1CallBack);
    ros::Subscriber w2_sub = node.subscribe("fantasmao/w2",1,w2CallBack);
    d_theta_i = 0;
    pub = node.advertise<geometry_msgs::Twist>("fantasmao/desSpeed",1);
    
    ros::Rate loopRate(180);
    
    while(ros::ok){
        ros::spinOnce();
        calculateSpeeds();
        loopRate.sleep();
    } 
    return 0;  
}