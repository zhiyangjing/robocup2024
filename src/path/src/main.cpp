#include <iostream>
#include <path/path.h>
#include <queue>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <stack>

#define TAG " [PATH]"

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

class Straight : public Ability {
private:
    bool is_running_ = false;

public:
    Straight(int remain_time, ros::NodeHandle nh) : Ability(remain_time, nh) {}
    void run() {
        is_running_ = true;
        int speed = 2;  // 默认速度是2
        int rate_num = 10;
        ros::Rate loop_rate(rate_num);  // 设置循环频率为10Hz

        ROS_INFO(TAG "Turning Left");
        nh_.getParam("speed", speed);
        nh_.getParam("direction", speed);
        nh_.setParam("direction", std::string(1, 'W'));
        nh_.setParam("angle", 0);  // 向左拐
        for (int i = 0; i < (5 * speed / 2) * rate_num; ++i) {
            loop_rate.sleep();
        }
        nh_.setParam("angle", 0);  // 回正
    }
    void stop() { is_running_ = false; }
};

class Reverse : public Ability {
private:
    bool is_running_ = false;

public:
    Reverse(int remain_time, ros::NodeHandle nh) : Ability(remain_time, nh) {}
    void run() {
        is_running_ = true;
        int speed = 2;  // 默认速度是2
        int rate_num = 10;
        ros::Rate loop_rate(rate_num);  // 设置循环频率为10Hz

        ROS_INFO(TAG "Turning Left");
        nh_.getParam("speed", speed);
        nh_.getParam("direction", speed);
        nh_.setParam("direction", std::string(1, 'S'));
        nh_.setParam("angle", 100);  // 向左拐
        for (int i = 0; i < (10 * speed / 2) * rate_num; ++i) {
            loop_rate.sleep();
        }
        nh_.setParam("angle", 0);  // 回正
    }
    void stop() { is_running_ = false; }
};

class RoadLeftTurn : public Ability {
private:
    bool is_running_ = false;

public:
    RoadLeftTurn(int remain_time, ros::NodeHandle nh) : Ability(remain_time, nh) {}
    void run() {
        is_running_ = true;
        int speed = 2;  // 默认速度是2
        int rate_num = 10;
        ros::Rate loop_rate(rate_num);  // 设置循环频率为10Hz

        ROS_INFO(TAG "Turning Left");
        nh_.getParam("speed", speed);
        nh_.setParam("angle", -200);  // 向左拐
        for (int i = 0; i < (8 * speed / 2) * rate_num; ++i) {
            loop_rate.sleep();
        }
        nh_.setParam("angle", 0);  // 回正
    }
    void stop() { is_running_ = false; }
};

class RoadRightTurn : public Ability {
private:
    bool is_running_ = false;

public:
    RoadRightTurn(int remain_time, ros::NodeHandle nh) : Ability(remain_time, nh) {};
    void run() {
        is_running_ = true;
        int speed = 2;  // 默认速度是2
        int rate_num = 10;
        ros::Rate loop_rate(rate_num);  // 设置循环频率为10Hz

        ROS_INFO(TAG "Turning Left");
        nh_.getParam("speed", speed);
        nh_.setParam("angle", 200);  // 向左拐
        for (int i = 0; i < (6 * speed / 2) * rate_num; ++i) {
            loop_rate.sleep();
        }
        nh_.setParam("angle", 0);  // 回正
    }
    void stop() { is_running_ = false; }
};

class Uturn : Ability {
public:
    Uturn(int remain_time, ros::NodeHandle nh) : Ability(remain_time, nh) { ROS_INFO(TAG "Uturn Ability Constructed"); }
    void run() {
        int speed = 2;  // 默认速度是2
        int rate_num = 10;
        ros::Rate loop_rate(rate_num);  // 设置循环频率为10Hz

        ROS_INFO(TAG "Turning right");
        nh_.getParam("speed", speed);
        nh_.setParam("angle", 100);  // 向右拐一点
        for (int i = 0; i < (2 * speed / 2) * rate_num; ++i) {
            loop_rate.sleep();
        }

        ROS_INFO(TAG "Turning left");
        nh_.setParam("angle", -200);  // 向左拐
        for (int i = 0; i < (8 * speed / 2) * rate_num; ++i) {
            loop_rate.sleep();
        }

        ROS_INFO(TAG "Goback back");
        nh_.setParam("direction", std::string(1, 'S'));
        nh_.setParam("angle", 200);  // 后退
        for (int i = 0; i < (4 * speed / 2) * rate_num; ++i) {
            loop_rate.sleep();
        }

        ROS_INFO(TAG "Keep on turn right");
        nh_.setParam("direction", std::string(1, 'W'));  // 改为前进
        nh_.setParam("angle", -200);

        for (int i = 0; i < (8 * speed / 2) * rate_num; ++i) {
            loop_rate.sleep();
        }
        nh_.setParam("angle", 0);  // 回正
    }
};

class PathController {
private:
    ros::NodeHandle &nh_;
    std::deque<int> states_queue;
    int STATE;

public:
    PathController(ros::NodeHandle nh) : nh_(nh) {
        // states_queue = std::deque<int>({LIGHT_DETECT,TRACE_LINE, ROAD_LEFT_TURN ,UTURN, TRACE_LINE, UTURN, TRACE_LINE, TERMINAL});
        states_queue = std::deque<int>(
            {TRACE_LINE, STRAIGHT, ROAD_LEFT_TURN, TRACE_LINE, ROAD_RIGHT_TURN, REVERSE, STRAIGHT, TERMINAL});
    }
    void start() {
        while (true) {
            STATE = states_queue.front();
            states_queue.pop_front();
            ROS_INFO(TAG "%s", string(20, '-').c_str());
            ROS_INFO(TAG "State: %d", STATE);
            ROS_INFO(TAG "%s", string(20, '-').c_str());
            if (STATE == LIGHT_DETECT) {
                auto light_detector = LightDetector(-1, nh_);
                light_detector.run();
            } else if (STATE == TRACE_LINE) {
                auto trace_line_controller = TraceLine(-1, nh_);
                trace_line_controller.run();
            } else if (STATE == UTURN) {
                auto uturn = Uturn(-1, nh_);
                uturn.run();
            } else if (STATE == STRAIGHT) {
                auto straight = Straight(-1, nh_);
                straight.run();
            } else if (STATE == ROAD_LEFT_TURN) {
                auto road_left_turn = RoadLeftTurn(-1, nh_);
                road_left_turn.run();
            } else if (STATE == ROAD_RIGHT_TURN) {
                auto road_right_turn = RoadRightTurn(-1, nh_);
                road_right_turn.run();
            } else if (STATE == REVERSE) {
                auto reverse = Reverse(-1, nh_);
                reverse.run();
            } else if (STATE == BIG_LEFT_TURN) {
                auto motion_controller = BigLeftTurn(10000, nh_);
                motion_controller.run();
            } else if (STATE == TERMINAL) {
                nh_.setParam("speed", 0);
                nh_.setParam("angle", 0);
                ROS_INFO(TAG "States equals TERMIANL, node exit");
                break;
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
