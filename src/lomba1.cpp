//Masukin ke package behaviour. jadi node behav_lomba1_node
#include "ros/ros.h"
#include "std_msgs/String.h"
//#include "daho_behavior/lomba1.hpp"
#include "lomba1.hpp" //nanti dihapus
#include <sstream>

//Class implementation
behav_lomba1::behav_lomba1(double set_X, double set_Y){}
double behav_lomba1::calculate_psi(int state, double yawNow){}
double behav_lomba1::calculate_theta(int state, double panNow){}
void behav_lomba1::destination_plan(){}
void behav_lomba1::walk(){}
void behav_lomba1::mini_translation(){}

int main(int argc, char **argv)
{
  return 0;
}