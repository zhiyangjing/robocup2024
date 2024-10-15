#include <chrono>
#include <common_utils/common.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>

using namespace std;

int frame_height = 480;
int frame_width = 720;
int main(int argc, char **argv) {
    ros::init(argc, argv, "minimal_test");
    ros::NodeHandle nh;

    // 打开摄像头
    cv::VideoCapture cap(0);
    if (!cap.isOpened()) {
        ROS_ERROR("Failed to open the camera_node");
        return -1;
    }

    cv::Mat frame;
    while (ros::ok()) {
        cap >> frame;
        if (frame.empty()) {
            ROS_WARN("Empty frame received");
            break;
        }


        cv::resize(frame, frame, cv::Size(frame_width, frame_height));
        cv::imshow("camera_node Feed", frame);
        int key = cv::waitKey(30);
        if (key == 'q')
            break;

        ros::spinOnce();// 处理 ROS 事件
    }

    cap.release();
    cv::destroyAllWindows();
    return 0;
}
