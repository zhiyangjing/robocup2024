#include <iostream>
#include <ros/ros.h>

enum States { BIG_LEFT_TURN = 0, SMALL_LEFT_TURN , TERMINAL };
int STATE = BIG_LEFT_TURN;

class Ability {
public:
    virtual void run() = 0;
    Ability(int remain_time, ros::NodeHandle nh) : remain_time_(remain_time), nh_(nh) {};

protected:
    ros::NodeHandle nh_;
    int remain_time_;
};