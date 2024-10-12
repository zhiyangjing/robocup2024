#include <chrono>
#include <common_utils/common.h>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <path/path.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/LaserScan.h>

using namespace std;

class AvoidObstacle : public Ability {
private:
    ros::Subscriber sub_;
    float min_distance = 0.8;
    int frameCount = 0;
    std::chrono::steady_clock::time_point lastTime = std::chrono::steady_clock::now();
    int frame_height = 480;
    int frame_width = 720;
    int line_pos = frame_width * 0.422;
    double currentFps = 0;

public:
    AvoidObstacle(int remain_time, ros::NodeHandle &nh) : Ability(remain_time, nh) {}

    float findMinDistance(vector<float> ranges) {
        float mindistance = ranges[175];
        for (int i = 220;i < 230;i++) {
           if (ranges[i] < mindistance) {
            mindistance = ranges[i];
           }
        }
        return mindistance;
    }

    // 图像处理函数
    void laserCallback(const sensor_msgs::LaserScan::ConstPtr &msg) {
        if (findMinDistance(msg->ranges) > min_distance) {
            return;
        }
        else if (findMinDistance(msg->ranges) <= min_distance) 
    }

    void run() { sub_ = nh_.subscribe("image_topic", 1, &AvoidObstacle::laserCallback, this); }
    void stop() {
        sub_ = ros::Subscriber();
    }
    ~AvoidObstacle() {}
};

int main(){

}