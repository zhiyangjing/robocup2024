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

    ParkInitParams reverse_park_params = ParkInitParams(16, 3);

    // reverse_park_params.ref_points.row(0) << 609, 627, 612, 889, 876, 370, 370, 588, 737, 743, 559, 750, 720;
    // reverse_park_params.ref_points.row(1) << 363, 370, 360, 424, 444, 311, 303, 249, 470, 489, 270, 366, 333;
    // reverse_park_params.ref_points.row(2) << 124, 136, 113, 363, 369, -136, -128, 37, 200, 253, 30, 115, 45;
    // reverse_park_params.ref_value << 0, 0, 0, -200, -200, 200, 200, 100, -100, -150, 150, -200, -200;
    // reverse_park_params.weights << 1, 2, 1;

    reverse_park_params.ref_points.row(0) << 650, 650, 685, 685, 600, 560, 550, 760, 778, 760, 600, 670, 750, 710, 650,
        670;
    reverse_park_params.ref_points.row(1) << 331, 334, 328, 328, 301, 288, 206, 359, 360, 355, 233, 275, 325, 400, 377,
        401;
    reverse_park_params.ref_points.row(2) << 42, 48, 31, 26, -43, -70, -54, 95, 135, 154, -50, 33, 150, 40, 44, 30;
    reverse_park_params.ref_value << 0, 0, 0, 0, 200, 150, 100, -200, -150, -100, 100, 60, 30, -100, -60, -30;
    reverse_park_params.weights << 1, 1, 1;
    reverse_park_params.camera = "back";

    // 测试节点
    ros::init(argc, argv, "trace_line");
    ros::NodeHandle nh;
    reverse_park_params.first_stage_param = -2;
    nh.getParam("first_stage_param_reverse", reverse_park_params.first_stage_param);
    cv::namedWindow("camera_node Feed", cv::WINDOW_AUTOSIZE);
    auto reverse_park = Park(-1, nh, reverse_park_params);
    reverse_park.run();
    cv::destroyAllWindows();
    return 0;
}
