//Masukin ke package behaviour
#include "ros/ros.h"
#include "std_msgs/String.h"
//#include "daho_behavior/lomba1.hpp"
#include "../include/lomba1.hpp"
#include <sstream>

class lomba1_node{
  private:
    ros::NodeHandle nh;
    ros::Subscriber pyaw, head_pan_sub;

    ros::Publisher forward, rotate;
    ros::Publisher sideStep, reset, gain;

  public:
    forward = nh.advertise<std_msgs::Float32>("/rhoban/forward", 1);
    rotate = nh.advertise<std_msgs::Float32>("/rhoban/rotate", 1);
    sideStep = nh.advertise<std_msgs::Float32>("/rhoban/sideStep", 1);
    reset = nh.advertise<std_msgs::Bool>("/rhoban/reset", 1);
    gain = nh.advertise<std_msgs::Float32>("/rhoban/gain", 1);

    pyaw = nh.subscribe<std_msgs::Int32>("/yaw",10, &lomba1_node::readYaw, this);
    head_pan_sub = nh.subscribe<daho_controller::JointMsg>("/controller/head/pan/read", 100, &lomba1_node::readPan, this);
    void readYaw(){

    }
    void readPan(const std_msgs::Int32ConstPtr &msg){

    }

};

int main(int argc, char **argv)
{
  return 0;
}