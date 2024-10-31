#include <chrono>
#include <common_utils/common.h>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <path/path.h>
#include <ros/ros.h>

/**
 * @brief 这里只是一个测试用的main函数，实际上真正使用这里的功能是在main.cpp
 *
 */
int main(int argc, char **argv) {
    // 测试节点
    ros::init(argc, argv, "trace_line");
    ros::NodeHandle nh;
    cv::namedWindow("camera_node Feed", cv::WINDOW_AUTOSIZE);
    SidePark side_park(10, nh);
    side_park.run();
    cv::destroyAllWindows();
    return 0;
}