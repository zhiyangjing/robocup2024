#include <opencv2/opencv.hpp>
#include <iostream>
#include <ros/ros.h>
#include <chrono>

using namespace std;


int main(int argc, char **argv)
{
    ros::init(argc, argv, "camera_node");
    ros::NodeHandle nh;

    // 打开摄像头
    cv::VideoCapture cap(0); // 0 表示默认摄像头
    if (!cap.isOpened())
    {
        ROS_ERROR("Failed to open the camera_node");
        return -1;
    }
    else
    {
        cout << "Camera opened successfully." << endl;
    }

    // 设置摄像头的分辨率为 1080p (1920x1080) 和目标帧率
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 720);  // 设置分辨率宽度
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480); // 设置分辨率高度
    cap.set(cv::CAP_PROP_FPS, 33);           // 设置目标帧率为 30

    cv::Mat frame;
    cout << "Starting the video feed..." << endl; // 开始视频流

    int targetFps = 20;                               // 目标帧率
    int delay = 1000 / targetFps;                     // 计算每帧的延迟时间
    auto lastTime = std::chrono::steady_clock::now(); // 记录开始时间
    int frameCount = 0;                               // 统计帧数
    double currentFps = 0.0;

    cv::Scalar lower_red1(0, 100, 100);
    cv::Scalar high_red1(10, 255, 255);
    cv::Scalar lower_red2(160, 100, 100);
    cv::Scalar high_red2(180, 255, 255);
    cv::Mat mask1, mask2, red_mask;


    while (ros::ok())
    {
        // 捕获帧
        cap >> frame;
        if (frame.empty())
        {
            ROS_WARN("Empty frame received");
            break;
        }

        cv::Mat hsv_frame;
        cv::cvtColor(frame,hsv_frame,cv::COLOR_BGR2HSV);
        cv::inRange(hsv_frame,lower_red1,high_red1,mask1);
        cv::inRange(hsv_frame,lower_red2,high_red2,mask2);
        red_mask = mask1 | mask2;

        vector<vector<cv::Point>> contours;
        vector<cv::Vec4i> hierarchy;
        cv::findContours(red_mask, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        // 标注红色的区域
        cv::drawContours(frame, contours, -1, cv::Scalar(0, 255, 0), 2);

        // 计算帧率
        frameCount++;
        auto currentTime = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsed = currentTime - lastTime;
        if (elapsed.count() >= 1.0)
        { // 每秒更新一次帧率
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
        if (key == 'q')
        { // 按 'q' 键退出
            break;
        }

        ros::spinOnce(); // 处理 ROS 事件
    }

    // 释放资源
    cap.release();
    cv::destroyAllWindows();
    return 0;
}
