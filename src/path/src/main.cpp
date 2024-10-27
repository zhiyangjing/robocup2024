#include <csignal>
#include <iostream>
#include <path/path.h>
#include <queue>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <stack>

#define TAG " [PATH] "

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
            ROS_INFO(TAG "TIME REMAIN: %d ,TIME DELTA: %d", remain_time_, time_delta);
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
    int lasting_time = 7;

public:
    Straight(int remain_time, ros::NodeHandle nh) : Ability(remain_time, nh) {}
    void run() {
        is_running_ = true;
        int speed = 2;  // 默认速度是2
        int rate_num = 10;
        ros::Rate loop_rate(rate_num);  // 设置循环频率为10Hz

        ROS_INFO(TAG "Going Straight");

        nh_.getParam("speed", speed);

        nh_.setParam("direction", std::string(1, 'W'));
        nh_.setParam("angle", 0);
        ROS_INFO(TAG "Angle now is: %d", 0);

        ROS_INFO(TAG "Total Time: %ds , %d iteration", lasting_time, (lasting_time * speed / 2) * rate_num);
        for (int i = 0; i < (lasting_time * speed / 2) * rate_num; ++i) {
            ROS_INFO(TAG "iteration: %d", i);
            loop_rate.sleep();
        }
        nh_.setParam("angle", 0);  // 回正
    }
    void stop() { is_running_ = false; }
};

class RoadLeftTurn : public Ability {
private:
    bool is_running_ = false;
    int lasting_time = 10;
    vector<float> stages_time;

public:
    RoadLeftTurn(int remain_time, ros::NodeHandle nh) : Ability(remain_time, nh) {
        nh_.getParam("road_left_turn_times", stages_time);
    }
    void run() {
        is_running_ = true;
        int speed = 2;  // 默认速度是2
        int rate_num = 10;
        ros::Rate loop_rate(rate_num);  // 设置循环频率为10Hz
        ROS_INFO(TAG "Turning Left");
        nh_.getParam("speed", speed);

        nh_.setParam("angle", 0);
        for (int i = 0; i < static_cast<int>((stages_time[0] * speed / 2) * rate_num); ++i) {
            loop_rate.sleep();
        }

        nh_.setParam("angle", -200);
        for (int i = 0; i < static_cast<int>((stages_time[1] * speed / 2) * rate_num); ++i) {
            loop_rate.sleep();
        }

        nh_.setParam("angle", 0);
        for (int i = 0; i < static_cast<int>((stages_time[2] * speed / 2) * rate_num); ++i) {
            loop_rate.sleep();
        }
        nh_.setParam("angle", 0);  // 回正
    }
    void stop() { is_running_ = false; }
};

class RoadRightTurn : public Ability {
private:
    bool is_running_ = false;
    vector<float> stages_time;

public:
    RoadRightTurn(int remain_time, ros::NodeHandle nh) : Ability(remain_time, nh) {
        nh_.getParam("road_left_turn_times", stages_time);
    };
    void run() {
        is_running_ = true;
        int speed = 2;  // 默认速度是2
        int rate_num = 10;
        ros::Rate loop_rate(rate_num);  // 设置循环频率为10Hz

        ROS_INFO(TAG "Turning Right");
        nh_.getParam("speed", speed);

        ROS_INFO(TAG COLOR_BLUE "Going Right" COLOR_RESET);
        nh_.setParam("angle", 200);
        for (int i = 0; i < static_cast<int>((8 * speed / 2) * rate_num); ++i) {
            loop_rate.sleep();
        }
    }
    void stop() { is_running_ = false; }
};

class Avoid : Ability {
private:
    vector<float> stages_time;

public:
    Avoid(int remain_time, ros::NodeHandle nh) : Ability(remain_time, nh) {
        ROS_INFO(TAG COLOR_YELLOW "Avoid Ability Constructed" COLOR_RESET);
        nh_.getParam("avoid_times", stages_time);
    }
    void run() {
        int speed = 2;  // 默认速度是2
        int rate_num = 10;
        ros::Rate loop_rate(rate_num);  // 设置循环频率为10Hz

        ROS_INFO(TAG COLOR_BLUE "Going left" COLOR_RESET);
        nh_.setParam("angle", -200);  // 向左拐
        for (int i = 0; i < static_cast<int>((stages_time[0] * speed / 2) * rate_num); ++i) {
            loop_rate.sleep();
        }

        ROS_INFO(TAG COLOR_BLUE "Going Straight" COLOR_RESET);
        nh_.setParam("angle", 0);
        for (int i = 0; i < static_cast<int>((stages_time[1] * speed / 2) * rate_num); ++i) {
            loop_rate.sleep();
        }

        ROS_INFO(TAG COLOR_BLUE "Going Right" COLOR_RESET);
        nh_.setParam("angle", 200);
        for (int i = 0; i < static_cast<int>((stages_time[2] * speed / 2) * rate_num); ++i) {
            loop_rate.sleep();
        }

        ROS_INFO(TAG COLOR_BLUE "Going Left" COLOR_RESET);
        nh_.setParam("angle", -200);
        for (int i = 0; i < static_cast<int>((stages_time[3] * speed / 2) * rate_num); ++i) {
            loop_rate.sleep();
        }

        ROS_INFO(TAG COLOR_BLUE "Keep Straight" COLOR_RESET);
        nh_.setParam("angle", 0);
    }
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
    static PathController *instance;
    ParkInitParams reverse_park_params;

public:
    PathController(ros::NodeHandle nh) : nh_(nh) {
        // states_queue = std::deque<int>({LIGHT_DETECT,TRACE_LINE, ROAD_LEFT_TURN ,UTURN, TRACE_LINE, UTURN, TRACE_LINE, TERMINAL});
        // states_queue = std::deque<int>({TRACE_LINE, STRAIGHT, TRACE_LINE, AVOID, TRACE_LINE, ROAD_LEFT_TURN, TRACE_LINE,
        //                                 ROAD_RIGHT_TURN, REVERSE_PARK, TERMINAL});
        states_queue = std::deque<int>(
            {TRACE_LINE, AVOID, TRACE_LINE, ROAD_LEFT_TURN, TRACE_LINE, ROAD_RIGHT_TURN, REVERSE_PARK, TERMINAL});
        vector<int> order;

        if (nh.getParam("order", order)) {
            states_queue.assign(order.begin(), order.end());
        }

        instance = this;

        // 为了对其只好这么写了
        // 应该要重新修改。改为640，480
        reverse_park_params = ParkInitParams(11, 3);
        reverse_park_params.ref_points.row(0) << 609, 627, 612, 889, 876, 370, 370, 588, 737, 743, 559;
        reverse_park_params.ref_points.row(1) << 363, 370, 360, 424, 444, 311, 303, 249, 470, 489, 270;
        reverse_park_params.ref_points.row(2) << 124, 136, 113, 363, 369, -136, -128, 37, 200, 253, 30;
        reverse_park_params.ref_value << 0, 0, 0, -200, -200, 200, 200, 100, -100, -150, 150;
        reverse_park_params.weights << 1, 1, 1;
    }

    static void signalHandler(int signum) {
        if (instance) {
            instance->handleShutdown();
        }
    }

    void handleShutdown() {
        ROS_INFO(TAG COLOR_RED "Ctrl + C detected. Cleaning up..." COLOR_RESET);
        states_queue.clear();
        STATE = TERMINAL;
        nh_.setParam("speed", 0);
        nh_.setParam("angle", 0);
        ROS_INFO(TAG "State set to TERMINAL. Queue cleared.");
        exit(0);
    }
    void start() {
        signal(SIGINT, PathController::signalHandler);
        ros::Rate rate(10);  // 每秒循环10次
        while (true) {
            if (states_queue.empty()) {
                break;
            }
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
            } else if (STATE == AVOID) {
                auto avoid = Avoid(-1, nh_);
                avoid.run();
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
            } else if (STATE == REVERSE_PARK) {
                auto reverse = Park(-1, nh_, reverse_park_params);
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
            ros::spinOnce();  // 执行一次ROS的回调
            rate.sleep();     // 控制循环频率
        }
    }
};

PathController *PathController::instance = nullptr;
int main(int argc, char *argv[]) {
    ros::init(argc, argv, "path_controller");
    ros::NodeHandle nh;
    PathController path_controller(nh);
    path_controller.start();
    return 0;
}
