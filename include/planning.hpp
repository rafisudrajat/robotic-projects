#ifndef PLANNING_STATE
#define PLANNING_STATE
#include "state.hpp"

class PLANNING: public State{
    private:
        ros::NodeHandle nh;
        ros::ServiceClient get_set;
    public:
        PLANNING(); //ctor
        void process();
        shared_ptr<State> checkCond();  
};