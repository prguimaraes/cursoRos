#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

#define b 0.35
#define r 0.1

float calculate_rightSpeed(float v, float w){
    return (v + b*w/2)/r;
}


float calculate_leftSpeed(float v, float w){
    return (v - b*w/2)/r;
}

int kbhit(void)
{
  struct termios oldt, newt;
  int ch;
  int oldf;
 
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
  fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);
 
  ch = getchar();
 
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  fcntl(STDIN_FILENO, F_SETFL, oldf);
 
  if(ch != EOF)
  {
    ungetc(ch, stdin);
    return 1;
  }
 
  return 0;
}

int main(int argc, char *argv[]){
    ros::init(argc, argv, "kine_controller");
    ros::NodeHandle node;
    ros::Publisher leftSpeed = node.advertise<std_msgs::Float64>("vrep/vehicle/motorLeftSpeed",1);
    ros::Publisher rightSpeed = node.advertise<std_msgs::Float64>("vrep/vehicle/motorRightSpeed",1);
    ros::Rate loop_rate(30);
    
    float v = 1;
    float w = 0;
    std_msgs::Float64 leftMsg;
    std_msgs::Float64 rightMsg;
    char aux;
    while(ros::ok()){
        if (kbhit()){
            std::cin >> aux;
            if(aux == 'w'){
                v = 1;
                w = 0;
            }
            if(aux == 'a'){
                v = 0;
                w = 2;
            }
            if(aux == 's'){
                v = -1;
                w = 0;
            }
            if(aux == 'd'){
                v = 0;
                w = -2;
            }
            if(aux == 'e'){
                v = 1;
                w = -2;
            }
            if(aux == 'q'){
                v = 1;
                w = 2;
            }
        }
        else{
            v = 0;
            w = 0;
        }
        leftMsg.data = calculate_leftSpeed(v,w);
        rightMsg.data = calculate_rightSpeed(v,w);
        leftSpeed.publish(leftMsg);
        rightSpeed.publish(rightMsg);
        ros::spinOnce();
        loop_rate.sleep();
    }
    
}