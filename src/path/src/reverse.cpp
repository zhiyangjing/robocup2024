#include <iostream>
#include <path/path.h>
#include <queue>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <stack>

#define TAG " [REVERSE]"

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