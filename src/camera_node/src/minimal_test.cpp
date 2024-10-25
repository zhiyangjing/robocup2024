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

    int targetFps = 20;                                // 目标帧率
    int delay = 1000 / targetFps;                      // 计算每帧的延迟时间
    auto lastTime = std::chrono::steady_clock::now();  // 记录开始时间
    int frameCount = 0;                                // 统计帧数
    double currentFps = 0.0;
    int frame_rate = 5;
    nh.getParam("frame_rate", frame_rate);
    ros::Rate loop_rate(frame_rate);

    int camera_port = 0;
    nh.getParam("camera_port", camera_port);
    cv::VideoCapture cap("/dev/camera" + to_string(camera_port));  // 0 表示默认摄像头

    if (!cap.isOpened()) {
        ROS_ERROR("Failed to open the camera_node");
        return -1;
    }
    // cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);   // 设置宽度为720
    // cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);  // 设置高度为480
    cap.set(cv::CAP_PROP_FPS, 25);
    cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));



    cv::Mat frame;
    while (ros::ok()) {
        cap >> frame;
        if (frame.empty()) {
            ROS_WARN("Empty frame received");
            break;
        }

        frameCount++;
        auto currentTime = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsed = currentTime - lastTime;
        if (elapsed.count() >= 1.0) {  // 每秒更新一次帧率
            currentFps = frameCount / elapsed.count();
            frameCount = 0;
            lastTime = currentTime;
        }

        // 在帧上显示帧率
        cv::putText(frame, "FPS: " + std::to_string(static_cast<int>(currentFps)), cv::Point(10, 30),
                    cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2);


        cv::resize(frame, frame, cv::Size(frame_width, frame_height));
        cv::imshow("camera_node Feed", frame);
        int key = cv::waitKey(1);
        if (key == 'q')
            break;

        ros::spinOnce();// 处理 ROS 事件
        loop_rate.sleep();
    }

    cap.release();
    cv::destroyAllWindows();
    return 0;
}
