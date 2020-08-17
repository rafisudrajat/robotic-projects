// ================================================== //
//                      Created By                    //
//           Aditya Putra Santosa / 13517013          //
// ================================================== //

#include <iostream>
#include <string>
#include <ros/ros.h>
#include <ros/package.h>
#include "daho_behavior/allState.hpp"

using namespace std;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "state_manager");
    ros::NodeHandle nh;

    shared_ptr<State> state(new Searching());

    ros::Rate r(60);
    while (ros::ok())
    {
        ROS_INFO_STREAM((*state));
        state->process();
        state = state->checkCond();
        r.sleep();
        ros::spinOnce();
    }

    return 0;
}