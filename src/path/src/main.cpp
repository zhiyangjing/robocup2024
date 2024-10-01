#include <iostream>
#include <ros/ros.h>
#define TAG " [PATH]"

enum States { BIG_LEFT_TURN = 0, SMALL_LEFT_TURN };
int STATE = BIG_LEFT_TURN;

class Ability {
public:
    virtual void run() = 0;
    Ability(int remain_time, ros::NodeHandle nh) : remain_time_(remain_time), nh_(nh) {};

protected:
    ros::NodeHandle nh_;
    int remain_time_;
};

class BigLeftTurn : public Ability {
private:
    int loop_rate = 50;
    int time_delta = 1000 / loop_rate;

public:
    BigLeftTurn(int remain_time, ros::NodeHandle nh) : Ability(remain_time, nh) {}

    void setMotion() {
        nh_.setParam("angle", 97);
        nh_.setParam("speed", 4);
    }

    void endMotion() {
        nh_.setParam("angle", 0);
        nh_.setParam("speed", 2);
    }

    void run() {
        ros::Rate looprate(50);
        while (ros::ok() and remain_time_ > 0) {
            ROS_INFO("%s TIME REMAIN: %d ,TIME DELTA", TAG, remain_time_, time_delta);
            setMotion();
            looprate.sleep();
            remain_time_ -= time_delta;
        }
        endMotion();
    }
};

class PathController {
private:
    ros::NodeHandle &nh_;

public:
    PathController(ros::NodeHandle nh) : nh_(nh) {};
    void start() {
        while (true) {
            if (STATE == BIG_LEFT_TURN) {
                auto motion_controller = BigLeftTurn(10000, nh_);
                motion_controller.run();
            }
        }
    }
};

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "path_controller");
    ros::NodeHandle nh;
    PathController path_controller(nh);
    path_controller.start();
    ros::spin();
    return 0;
}
