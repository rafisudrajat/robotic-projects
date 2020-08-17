#ifndef STATE_HPP
#define STATE_HPP

#include <memory>
#include <utility>
#include <string>
#include <ros/ros.h>

using namespace std;

class State : public enable_shared_from_this<State>
{
protected:
    static int jumState;
    int id;
    string nama_state;
    ros::Time start;
    ros::NodeHandle nh;

public:
    State(string nama);
    virtual ~State();
    virtual void process();
    virtual shared_ptr<State> checkCond();

    int getId() { return id; }
    string getNama() { return nama_state; }

    ros::Duration elapsedTime() const;

    friend ostream & operator << (ostream &out, const State &s); 
};

#endif