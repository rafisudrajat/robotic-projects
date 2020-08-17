#include "../../include/state.hpp"
#include <iostream>
#include <ros/ros.h>

using namespace std;

int State::jumState = 0;

State::State(string nama)
{
    start = ros::Time::now();
    nama_state = nama;
    id = jumState++;
}

State::~State()
{
    cout << "DTOR state id(" << id << ") nama(" << nama_state << ")" << endl;
}

void State::process()
{
    ROS_ERROR_NAMED("State", "PROCESS STATE KOSONG, ini harusnya interface");
}

shared_ptr<State> State::checkCond()
{
    ROS_ERROR_NAMED("State", "CHECK KONDISI STATE KOSONG, ini harusnya interface");
    return shared_from_this();
}

ros::Duration State::elapsedTime() const // isi const untuk menjamin tidak ada perubahan variable didalam fungsi ini
{
    return ros::Time::now() - start;
}

ostream &operator<<(ostream &out, const State &c)
{
    out << "State id(" << c.id << "), nama(" << c.nama_state << "), elapsedTime(" << c.elapsedTime().toSec() << " second)" << endl;
    return out;
}
