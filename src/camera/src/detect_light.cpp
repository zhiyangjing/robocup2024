#include <chrono>
#include <common_utils/common.h>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

using namespace std;

class LightDetector {
private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;

    int frameCount = 0;
    std::chrono::steady_clock::time_point lastTime = std::chrono::steady_clock::now();
    int frame_height = 480;
    int frame_width = 720;
    int line_pos = frame_width * 0.422;
    double currentFps = 0;

    // HSV阈值设置
    cv::Scalar lower_red1 = cv::Scalar(0, 100, 100);
    cv::Scalar high_red1 = cv::Scalar(10, 255, 255);
    cv::Scalar lower_red2 = cv::Scalar(160, 100, 100);
    cv::Scalar high_red2 = cv::Scalar(180, 255, 255);
    cv::Scalar lower_green = cv::Scalar(40, 100, 100);
    cv::Scalar high_green = cv::Scalar(80, 255, 255);
    cv::Mat hsv_frame, green_mask;
    cv::Mat mask1, mask2, red_mask;

public:
    LightDetector(ros::NodeHandle &nh) : nh_(nh) {
        sub_ = nh_.subscribe("image_topic", 1, &LightDetector::imageCallback, this);
    }

    // 计算并显示帧率的函数
    void imageCallback(const sensor_msgs::ImageConstPtr &msg) {
        try {
            cv::Mat frame = cv_bridge::toCvShare(msg, "bgr8")->image;

            cv::resize(frame, frame, cv::Size(frame_width, frame_height));
            if (frame.empty()) {
                ROS_WARN("Empty frame received");
                return;
            }

            // 选择线条的颜色和粗细
            cv::Scalar color(0, 255, 0);// 绿色
            int thickness = 2;          // 线条宽度

            // 探测红绿灯
            detectTrafficLights(frame, nh_);

            // 计算并显示帧率
            displayFps(frame, frameCount, lastTime);

            // 计算并显示中心线
            int y_start = static_cast<int>(frame_height * 0.2);
            int y_end = static_cast<int>(frame_height * 0.8);
            cv::line(frame, cv::Point(line_pos, y_start), cv::Point(line_pos, y_end), color, thickness);
            cout << "[DEBUG]: start point" << line_pos << " ," << y_start << endl;
            cout << "[DEBUG]: end point" << line_pos << " ," << y_end << endl;

            cv::imshow("camera_node Feed", frame);
            cv::waitKey(30);

            ros::spinOnce();// 处理 ROS 事件
        } catch (cv_bridge::Exception & e) {
            ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
        }
    }

    // 探测红色和绿色区域的函数
    void detectTrafficLights(cv::Mat &frame, ros::NodeHandle &nh) {
        cv::cvtColor(frame, hsv_frame, cv::COLOR_BGR2HSV);

        // 生成掩码
        ROS_INFO("[DEBUG TAG] 1");
        cv::inRange(hsv_frame, lower_red1, high_red1, mask1);
        cv::inRange(hsv_frame, lower_red2, high_red2, mask2);
        red_mask = mask1 | mask2;
        cv::inRange(hsv_frame, lower_green, high_green, green_mask);

        vector<vector<cv::Point>> contours_red, contours_green;
        vector<cv::Vec4i> hierarchy_red, hierarchy_green;

        ROS_INFO("[DEBUG TAG] 2");
        cv::findContours(red_mask, contours_red, hierarchy_red, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        cv::findContours(green_mask, contours_green, hierarchy_green, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        double maxRedArea = 0, maxGreenArea = 0;
        int maxRedIdx = -1, maxGreenIdx = -1;

        ROS_INFO("[DEBUG TAG] 3");
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

        ROS_INFO("[DEBUG TAG] 4");
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
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "detect_light");
    ros::NodeHandle nh;
    cv::namedWindow("camera_node Feed", cv::WINDOW_AUTOSIZE);    

    LightDetector lightProcessor(nh);

    ros::spin();
    cv::destroyAllWindows();
    return 0;
}