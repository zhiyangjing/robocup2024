#include <opencv2/opencv.hpp>
#include <iostream>
#include <ros/ros.h>
#include <chrono>

using namespace std;

int main(int argc, char** argv) {
    ros::init(argc, argv, "camera_node");
    ros::NodeHandle nh;

    // 打开摄像头
    cv::VideoCapture cap(0);  // 0 表示默认摄像头
    if (!cap.isOpened()) {
        ROS_ERROR("Failed to open the camera_node");
        return -1;
    } else {
        cout << "Camera opened successfully." << endl;
    }

    // 设置摄像头的分辨率为 1080p (1920x1080) 和目标帧率
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 1080);  // 设置分辨率宽度
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 720); // 设置分辨率高度
    cap.set(cv::CAP_PROP_FPS, 33);            // 设置目标帧率为 30

    cv::Mat frame;
    cout << "Starting the video feed..." << endl;  // 开始视频流

    int targetFps = 30;  // 目标帧率
    int delay = 1000 / targetFps;  // 计算每帧的延迟时间
    auto lastTime = std::chrono::steady_clock::now();  // 记录开始时间
    int frameCount = 0;  // 统计帧数
    double currentFps = 0.0;

    while (ros::ok()) {
        // 捕获帧
        cap >> frame;
        if (frame.empty()) {
            ROS_WARN("Empty frame received");
            break;
        }

        // 计算帧率
        frameCount++;
        auto currentTime = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsed = currentTime - lastTime;
        if (elapsed.count() >= 1.0) {  // 每秒更新一次帧率
            currentFps = frameCount / elapsed.count();
            frameCount = 0;
            lastTime = currentTime;
        }

        // 在帧上显示帧率
        cv::putText(frame, "FPS: " + std::to_string(static_cast<int>(currentFps)),
                    cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 
                    1, cv::Scalar(0, 255, 0), 2);

        // 显示捕获的帧
        cv::imshow("camera_node Feed", frame);

        // 使用 waitKey 控制帧率和响应键盘输入
        int key = cv::waitKey(delay);
        if (key == 'q') {  // 按 'q' 键退出
            break;
        }

        ros::spinOnce();  // 处理 ROS 事件
    }

    // 释放资源
    cap.release();
    cv::destroyAllWindows();
    return 0;
}
