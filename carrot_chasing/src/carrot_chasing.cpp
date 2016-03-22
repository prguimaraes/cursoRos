#include <iostream>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <math.h>

#define T 1/32

#define Kp 4
#define Kd 200 
#define Ki 0
#define K2 2

#define X 0
#define Y 1

ros::Publisher pub;
geometry_msgs::Twist robot_speeds;

double w1[2] = {4, 1.5};
double w2[2] = {-3, 1.5};

double d_theta[2];
double d_theta_i;

double delta = 0.2;

double normalizeAngles(double angle){
    if(angle > M_PI){
        angle = angle - 2*M_PI;
    }
    if (angle < -M_PI){
        angle = angle + 2*M_PI;
    }
    return angle;
}

double calculateDistance(double *p1, double *p2){
    return sqrt(pow(p1[X] - p2[X],2) + pow(p1[Y] - p2[Y],2));
}

void callback(const nav_msgs::OdometryConstPtr &msg){
    double p[2];
    p[X] = msg->pose.pose.position.x;
    p[Y] = msg->pose.pose.position.y;
    
    double Ru = calculateDistance(w1,p);
    double Theta_u = atan2(p[Y] - w1[Y], p[X] - w1[X]);
    double Theta = atan2(w2[Y] - w1[Y], w2[X] - w1[X]);
    double Beta = Theta - Theta_u;
    double R = sqrt(pow(Ru,2) - pow(Ru*sin(Beta),2));
    double c[2];
    
    c[X] = (R + delta)*cos(Theta) + w1[X];
    c[Y] = (R + delta)*sin(Theta) + w1[Y];
    
    double dist = calculateDistance(w2,p);
    
    double psi = tf::getYaw(msg->pose.pose.orientation);
    double psi_d = atan2(c[Y] - p[Y],c[X] - p[X]);
    
    d_theta[1] = d_theta[0];
    d_theta[0] = normalizeAngles(psi_d - psi);
    d_theta_i = d_theta_i + d_theta[0];
    
    robot_speeds.linear.x = 1.5;
    robot_speeds.angular.z = Kp*d_theta[0] + Kd*(d_theta[0] - d_theta[1])/T + Ki*d_theta_i;
    
//    if(dist > 0.1 && dist <= 0.5){
//        robot_speeds.linear.x = K2*dist;
//        robot_speeds.angular.z = K1*d_theta;
//    }
//    if(dist > 0.5){
//        robot_speeds.linear.x = 1.5;
//        robot_speeds.angular.z = K1*d_theta;
//
//    }
//    else{
//        robot_speeds.linear.x = 0;
//        robot_speeds.angular.z = 0;
//    }
    ROS_INFO("d = %f, x = %f, y = %f",dist,p[X],p[Y]);
    pub.publish(robot_speeds);
}

int main(int argc, char *argv[]){
    ros::init(argc, argv, "fantasmao_carrot");
    ros::NodeHandle node;
    ros::Subscriber sub = node.subscribe("vrep/vehicle/odometry",1,callback);
    d_theta_i = 0;
    pub = node.advertise<geometry_msgs::Twist>("fantasmao/desSpeed",1);
    
    ros::spin();    
}