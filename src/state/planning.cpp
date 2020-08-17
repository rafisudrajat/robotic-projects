#include "../../include/planning.hpp"
#include <ros/ros.h>
#include "Package_Name/setpoint.h" //Package_Name diubah nantinya 

PLANNING::PLANNING():State("Planning"){
    get_set = nh.serviceClient<Package_Name::setpoint>("get_setpoint");
}

PLANNING::process() {
    Package_Name::setpoint srv;
    get_set.call(srv);
}

shared_ptr<State> PLANNING::checkCond()
{
    return make_shared<muter>();
}        
