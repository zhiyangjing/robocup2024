#include <chrono>
#include <common_utils/common.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>

using namespace std;

// HSV阈值设置
cv::Scalar lower_red1(0, 100, 100);
cv::Scalar high_red1(10, 255, 255);
cv::Scalar lower_red2(160, 100, 100);
cv::Scalar high_red2(180, 255, 255);
cv::Scalar lower_green(40, 100, 100);
cv::Scalar high_green(80, 255, 255);

cv::Mat hsv_frame, green_mask;
cv::Mat mask1, mask2, red_mask;
// 探测红色和绿色区域的函数
void detectTrafficLights(cv::Mat &frame, ros::NodeHandle &nh) {
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
        cv::drawContours(frame, contours_red, maxRedIdx, cv::Scalar(0, 255, 0), 2);// 用绿色围住红色区域
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
        cv::drawContours(frame, contours_green, maxGreenIdx, cv::Scalar(0, 0, 255), 2);// 用红色围住绿色区域
    }

    // 根据最大区域的大小设置light参数
    if (maxGreenArea > maxRedArea) {
        nh.setParam("light", TrafficLight::GREEN);
        ROS_INFO("Changed into " COLOR_GREEN "green" COLOR_RESET);
    } else {
        nh.setParam("light", TrafficLight::RED);
        ROS_INFO("Changed into " COLOR_RED "red" COLOR_RESET);
    }
}

double currentFps = 0;
// 计算并显示帧率的函数
void displayFps(cv::Mat &frame, int &frameCount, std::chrono::steady_clock::time_point &lastTime) {
    frameCount++;
    auto currentTime = std::chrono::steady_clock::now();
    std::chrono::duration<double> elapsed = currentTime - lastTime;

    if (elapsed.count() >= 1.0) {
        currentFps = frameCount / elapsed.count();
        frameCount = 0;
        lastTime = currentTime;

        // 在帧上显示帧率
    }
    cv::putText(frame, "FPS: " + std::to_string(static_cast<int>(currentFps)), cv::Point(10, 30),
                cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "camera_node");
    ros::NodeHandle nh;

    // 打开摄像头
    cv::VideoCapture cap(0);
    if (!cap.isOpened()) {
        ROS_ERROR("Failed to open the camera_node");
        return -1;
    }

    // 会造成严重问题
    // cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
    // cap.set(cv::CAP_PROP_FRAME_WIDTH, 720);
    // cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
    // cap.set(cv::CAP_PROP_FPS, 33);

    cv::Mat frame;
    int targetFps = 20;
    int delay = 1000 / targetFps;
    int frameCount = 0;
    auto lastTime = std::chrono::steady_clock::now();
    int frame_height = 720;
    int frame_width = 480;
    int line_pos = frame_width * 0.47;
    while (ros::ok()) {
        cap >> frame;
        cv::resize(frame, frame, cv::Size(frame_height, frame_width));
        if (frame.empty()) {
            ROS_WARN("Empty frame received");
            break;
        }

        // 选择线条的颜色和粗细
        cv::Scalar color(0, 255, 0);// 绿色
        int thickness = 2;          // 线条宽度

        // 探测红绿灯
        detectTrafficLights(frame, nh);

        // 计算并显示帧率
        displayFps(frame, frameCount, lastTime);
        int y_start = static_cast<int>(frame_height * 0.2);
        int y_end = static_cast<int>(frame_height * 0.8);
        cv::line(frame, cv::Point(line_pos, y_start), cv::Point(line_pos, y_end), color, thickness);

        cv::imshow("camera_node Feed", frame);
        int key = cv::waitKey(delay);
        // if (key == 'q')
        //     break;

        ros::spinOnce();// 处理 ROS 事件
    }

    cap.release();
    cv::destroyAllWindows();
    return 0;
}
