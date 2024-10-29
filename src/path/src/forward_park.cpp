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

    ParkInitParams forward_park_params = ParkInitParams(16, 3);

    forward_park_params.ref_points.row(0) << 640, 645, 644, 660, 790, 761, 798, 598, 580, 570, 595, 570, 552, 740, 730,
        662;
    forward_park_params.ref_points.row(1) << 410, 418, 410, 401, 446, 429, 428, 379, 378, 379, 278, 264, 270, 554, 493,
        460;
    forward_park_params.ref_points.row(2) << 120, 130, 131, 139, 226, 180, 226, 50, 35, 60, 10, 0, 20, 230, 185, 130;
    forward_park_params.ref_value << 0, 0, 0, 0, 200, 150, 100, -200, -150, -100, -200, -120, -60, 200, 120, 60;

    forward_park_params.weights << 1, 1, 1;
    forward_park_params.first_stage_param = 2;
    forward_park_params.camera = "front";
    forward_park_params.lowerFraction = 0.4;

    // 测试节点
    ros::init(argc, argv, "trace_line");
    ros::NodeHandle nh;
    forward_park_params.first_stage_param = -2;
    nh.getParam("first_stage_param_forward", forward_park_params.first_stage_param);
    cv::namedWindow("camera_node Feed", cv::WINDOW_AUTOSIZE);
    auto forward_park = Park(-1, nh, forward_park_params);
    forward_park.run();
    cv::destroyAllWindows();
    return 0;
}
