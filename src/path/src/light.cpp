#include <chrono>
#include <common_utils/common.h>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <path/path.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

using namespace std;

#define TAG " [Light] "

LightDetector::LightDetector(int remain_time, ros::NodeHandle &nh) : Ability(remain_time, nh) {
    ROS_INFO(TAG "LightDetector Constructed");
    light_states = Buffer<int>(5);

    int video_feed_back_param = 1;
    nh_.getParam("video_feed_back", video_feed_back_param);
    video_feed_back = (video_feed_back_param == 1);
    ROS_INFO(TAG COLOR_MAGENTA "Video feed back %s" COLOR_RESET,
             video_feed_back ? (COLOR_RED "Enabled") : (COLOR_GREEN "Disable"));
}

void LightDetector::run() {
    is_running_ = true;
    sub_ = nh_.subscribe("/image_topic/front", 1, &LightDetector::imageCallback, this);

    ros::Rate rate(30);
    while (ros::ok() && is_running_) {
        ros::spinOnce();
        rate.sleep();
    }
}

void LightDetector::stop() {
    is_running_ = false;
    sub_.shutdown();
    ROS_INFO(TAG "Light detector Ability exit");
}

void LightDetector::imageCallback(const sensor_msgs::ImageConstPtr &msg) {
    try {
        cv::Mat frame = cv_bridge::toCvShare(msg, "bgr8")->image;

        cv::resize(frame, frame, cv::Size(frame_width, frame_height));
        if (frame.empty()) {
            ROS_WARN("Empty frame received");
            return;
        }

        // 选择线条的颜色和粗细
        cv::Scalar color(0, 255, 0);  // 绿色
        int thickness = 2;            // 线条宽度

        // 探测红绿灯
        detectTrafficLights(frame, nh_);

        // 计算并显示中心线
        int y_start = static_cast<int>(frame_height * 0.2);
        int y_end = static_cast<int>(frame_height * 0.8);
        cv::line(frame, cv::Point(line_pos, y_start), cv::Point(line_pos, y_end), color, thickness);

        if (video_feed_back) {
            cv::imshow("camera_node Feed", frame);
            cv::waitKey(10);
        }

        ros::spinOnce();  // 处理 ROS 事件
    } catch (cv_bridge::Exception &e) { ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str()); }
}

// 探测红色和绿色区域的函数
void LightDetector::detectTrafficLights(cv::Mat &frame, ros::NodeHandle &nh) {
    cv::cvtColor(frame, hsv_frame, cv::COLOR_BGR2HSV);

    // 生成掩码
    cv::inRange(hsv_frame, lower_red1, high_red1, mask1);
    cv::inRange(hsv_frame, lower_red2, high_red2, mask2);
    red_mask = mask1 | mask2;
    cv::inRange(hsv_frame, lower_green, high_green, green_mask);

    vector<vector<cv::Point>> contours_red, contours_green;
    vector<cv::Vec4i> hierarchy_red, hierarchy_green;

    cv::findContours(red_mask, contours_red, hierarchy_red, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    cv::findContours(green_mask, contours_green, hierarchy_green, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    double maxRedArea = 0, maxGreenArea = 0;
    int maxRedIdx = -1, maxGreenIdx = -1;

    // 查找最大红色区域
    for (size_t i = 0; i < contours_red.size(); i++) {
        double area = cv::contourArea(contours_red[i]);
        if (area > maxRedArea) {
            maxRedArea = area;
            maxRedIdx = i;
        }
    }
    if (maxRedIdx != -1) {
        cv::drawContours(frame, contours_red, maxRedIdx, cv::Scalar(0, 255, 0), 2);  // 用绿色围住红色区域
    }

    // 查找最大绿色区域
    for (size_t i = 0; i < contours_green.size(); i++) {
        double area = cv::contourArea(contours_green[i]);
        if (area > maxGreenArea) {
            maxGreenArea = area;
            maxGreenIdx = i;
        }
    }

    if (maxGreenIdx != -1) {
        cv::drawContours(frame, contours_green, maxGreenIdx, cv::Scalar(0, 0, 255), 2);  // 用红色围住绿色区域
    }

    // 根据最大区域的大小设置light参数
    if (maxGreenArea > maxRedArea) {
        nh.setParam("light", TrafficLight::GREEN);
        light_states.push(GREEN);
        ROS_INFO("Changed into " COLOR_GREEN "green" COLOR_RESET);
    } else {
        nh.setParam("light", TrafficLight::RED);
        light_states.push(RED);
        ROS_INFO("Changed into " COLOR_RED "red" COLOR_RESET);
    }
    if (light_states.avg() >= 1) {
        stop();
    }
}

// int main(int argc, char **argv) {
//     ros::init(argc, argv, "detect_light");
//     ros::NodeHandle nh;
//     cv::namedWindow("camera_node Feed", cv::WINDOW_AUTOSIZE);

//     LightDetector lightProcessor(-1, nh);

//     ros::spin();
//     cv::destroyAllWindows();
//     return 0;
// }