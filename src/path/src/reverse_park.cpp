#include <iostream>
#include <path/path.h>
#include <queue>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/LaserScan.h>
#include <stack>

/**
 * @brief 这里只是一个测试用的main函数，实际上真正使用这里的功能是在main.cpp
 *
 */
int main(int argc, char **argv) {
    // 测试节点
    ros::init(argc, argv, "trace_line");
    ros::NodeHandle nh;
    cv::namedWindow("camera_node Feed", cv::WINDOW_AUTOSIZE);

    Park reverse_park(10, nh);
    reverse_park.run();
    cv::destroyAllWindows();
    return 0;
}
