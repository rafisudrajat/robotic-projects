#include <iostream>
#include <stdlib.h>
#include <stdint.h>
#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include <algorithm>
#include <memory>

using namespace std;

int stats;
int yaw, pitch, roll;
int yaw_offset, pitch_offset, roll_offset;
ros::Subscriber sub, suby, subp, subr;
ros::Publisher puby, pubp, pubr;

void chatterCallbackStats(const std_msgs::Int32::ConstPtr& msg)
{
    stats = msg->data;
    if (stats == 1) {
      yaw_offset = yaw;
      pitch_offset = pitch;
      roll_offset = roll;
    }
}

void chatterCallbackYaw(const std_msgs::Int32::ConstPtr& msg)
{
  yaw = (msg->data - yaw_offset) % 360;
  if (yaw < 0) {
    yaw = yaw + 360;
  }
}  


void chatterCallbackPitch(const std_msgs::Int32::ConstPtr& msg)
{
    pitch = (msg->data - pitch_offset) % 360;
    if (pitch < 0) {
      pitch = pitch + 360;
    }
  // if (stats == 1) {
  //   pitch_offset = pitch;
  // } else {
  //   if (pitch >= pitch_offset) {
  //     pitch = pitch - pitch_offset;
  //   } else if (pitch < pitch_offset) {
  //     pitch = pitch_offset - pitch;
  //   }
  // }
}

void chatterCallbackRoll(const std_msgs::Int32::ConstPtr& msg)
{
    roll = (msg->data - roll_offset) % 360;
    if (roll < 0) {
    roll = roll + 360;
    }
  // if (stats == 1) {
  //   roll_offset = roll;
  // } else {
  //   if (roll >= roll_offset) {
  //     roll = roll - roll_offset;
  //   } else if (roll < roll_offset) {
  //     roll = roll_offset - roll;
  //   }
  // }
}

void serialPublisherCb(const ros::TimerEvent)
{
    std_msgs::Int32 tmp;
    tmp.data = yaw;
    puby.publish(tmp);
    tmp.data = pitch;
    pubp.publish(tmp);
    tmp.data = roll;
    pubr.publish(tmp);
    cout << "yaw : " << yaw << endl;
    cout << "pitch : " << pitch << endl;
    cout << "roll : " << roll << endl;
    cout << "dummy : " << stats << endl;
}

class store_setpoint
{
    private:        
        int setpoint_x, setpoint_y;
        int current_setpoint;
    public:
        store_setpoint(int start_yaw){
            setpoint_x=start_yaw;
            setpoint_y=setpoint_x+90;
        }
        void change_setpoint(Package_Name::setpoint::Response &res)
        {
            if(current_setpoint==setpoint_y)
                current_setpoint=setpoint_x;
            else
                current_setpoint=setpoint_y;
            res.setpoint=current_setpoint;
        }
}

int main(int argc, char **argv)
{   
    
    ros::init(argc, argv, "set_point_node");
    ros::NodeHandle n;
    ros::Timer serialPublisher;

    ros::ServiceServer service = n.advertiseService("get_setpoint", &store_setpoint::change_setpoint, &store_setpoint);
    
    sub = n.subscribe("/set_point", 10, chatterCallbackStats);
    suby = n.subscribe("/yaw", 10, chatterCallbackYaw);
    subp = n.subscribe("/pitch", 10, chatterCallbackPitch);
    subr = n.subscribe("/roll", 10, chatterCallbackRoll);
    store_setpoint setting_setpoint(yaw);
    
    {
    ros::init(argc, argv, "set_point_node");
    ros::NodeHandle n;
    ros::Timer serialPublisher;

    ros::ServiceServer service = n.advertiseService("get_setpoint", &store_setpoint::change_setpoint, &store_setpoint);
    
    sub = n.subscribe("/set_point", 10, chatterCallbackStats);
    suby = n.subscribe("/yaw", 10, chatterCallbackYaw);
    subp = n.subscribe("/pitch", 10, chatterCallbackPitch);
    subr = n.subscribe("/roll", 10, chatterCallbackRoll);
    puby = n.advertise<std_msgs::Int32>("/yaw_offset", 10);
    pubp = n.advertise<std_msgs::Int32>("/pitch_offset", 10);
    pubr = n.advertise<std_msgs::Int32>("/roll_offset", 10);
    serialPublisher = n.createTimer(ros::Duration(1.0 / 120), serialPublisherCb);

    ros::spin();
    return 0;
    }
    
}