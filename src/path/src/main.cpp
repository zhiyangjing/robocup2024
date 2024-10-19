#include <iostream>
#include <path/path.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <stack>

#define TAG " [PATH]"
int STATE = BIG_LEFT_TURN;

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
            ROS_INFO("%s TIME REMAIN: %d ,TIME DELTA: %d", TAG, remain_time_, time_delta);
            setMotion();
            looprate.sleep();
            remain_time_ -= time_delta;
        }
        endMotion();
    }
};

class RoadLeftTurn : public Ability {
private:
    bool is_running_ = false;

public:
    RoadLeftTurn(int remain_time, ros::NodeHandle nh) : Ability(remain_time, nh) {}
    void run() {
        is_running_ = true;
        int speed = 2;  // 默认速度是2；
        nh_.getParam("speed", speed);
        int default_time = 3000000 * 2 / speed;
        nh_.setParam("angle", -200);
        usleep(default_time);
        stop();
    }
    void stop() { is_running_ = false; }
};

class Uturn : Ability {

public:
    Uturn(int remain_time, ros::NodeHandle nh) : Ability(remain_time, nh) {}
    void run() {
        int speed = 2;  // 默认速度是2，所有参数，在调试的时候使用速度2来测试
        nh_.getParam("speed", speed);
        nh_.setParam("angle", 100);  // 向向右拐一点
        usleep(500000 * speed / 2);
        nh_.setParam("angle", -200);  // 向左拐
        usleep(1000000 * speed / 2);
        nh_.setParam("angle", 0);
        nh_.setParam("direction", std::string(1, 'S'));  // 后退
        usleep(1000000 * speed / 2);
        nh_.setParam("direction", std::string(1, 'W'));  // 改为前进
        nh_.setParam("angle", -200);
        usleep(1000000 * speed / 2);
        nh_.setParam("angle", 0);  // 回正
    }
};

class PathController {
private:
    ros::NodeHandle &nh_;
    std::stack<int> states_stack;

public:
    PathController(ros::NodeHandle nh) : nh_(nh) {
        states_stack = stack<int>({
            LIGHT_DETECT, 
            TRACE_LINE, 
            UTURN,
            TERMINAL
        });
    }
    void start() {
        while (true) {
            if (STATE == LIGHT_DETECT) {
                auto light_dector = LightDetector(-1, nh_);
                light_dector.run();
            } else if (STATE == TRACE_LINE) {
                auto trace_line_controller = TraceLine(-1, nh_);
                trace_line_controller.run();
            } else if (STATE == UTURN) {
                auto uturn = Uturn(-1,nh_);
                uturn.run();
            } else if (STATE == ROAD_LEFT_TURN) {
                auto road_left_turn = RoadLeftTurn(-1, nh_);
                road_left_turn.run();
            } else if (STATE == BIG_LEFT_TURN) {
                auto motion_controller = BigLeftTurn(10000, nh_);
                motion_controller.run();
            } else if (STATE == TERMINAL) {
                ROS_INFO(TAG "States equals TERMIANL, node exit");
                exit(0);
            }
            STATE = states_stack.top();
            states_stack.pop();
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
