#include <iostream>
#include <ros/ros.h>

enum States { BIG_LEFT_TURN = 0, SMALL_LEFT_TURN };
int STATE = BIG_LEFT_TURN;

class Ability {
public:
    static ros::NodeHandle nh_;
    virtual void run() = 0;
    Ability(int remain_time) : remain_time_(remain_time) {};

protected:
    int remain_time_;
};

ros::NodeHandle Ability::nh_;

class BigLeftTurn : public Ability {
private:
    int loop_rate = 50;
    int time_delta = 1000 / loop_rate;

public:
    BigLeftTurn(int remain_time) : Ability(remain_time) {}

    void setMotion() {
        nh_.setParam("angle", 97);
        nh_.setParam("speed", 4);
    }

    void run() {
        ros::Rate looprate(50);
        while (ros::ok() and remain_time_ > 0) {
            setMotion();
            looprate.sleep();
            remain_time_ -= time_delta;
        }
    }
};

class PathController {
public:
    PathController() {};
    void start() {
        while (true) {
            if (STATE == BIG_LEFT_TURN) {
                auto motion_controller = BigLeftTurn(10);
                motion_controller.run();
            }
        }
    }
};

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "path_controller");

    PathController path_controller;
    path_controller.start();
    return 0;
}
