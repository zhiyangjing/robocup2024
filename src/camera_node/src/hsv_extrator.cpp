#include <chrono>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#define TAG "[hsv_extrator]"
using namespace std;

int h_min = 0, s_min = 0, v_min = 0;
int h_max = 179, s_max = 255, v_max = 255;
cv::Mat hsv_frame, mask;// 全局变量，存储图像和处理结果
int img_height = 480;
int img_width = 720;

void on_trackbar(int, void *) {
    cv::inRange(hsv_frame, cv::Scalar(h_min, s_min, v_min), cv::Scalar(h_max, s_max, v_max), mask);
    cv::imshow("Mask", mask);// 显示二值化的掩码
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "hsv_extrator");
    ros::NodeHandle nh;

    // 创建窗口和滑动条
    cv::namedWindow("Camera Feed", cv::WINDOW_AUTOSIZE);
    cv::namedWindow("Mask", cv::WINDOW_AUTOSIZE);

    // 创建滑动条，调节 HSV 值
    cv::createTrackbar("H Min", "Mask", &h_min, 179, on_trackbar);
    cv::createTrackbar("H Max", "Mask", &h_max, 179, on_trackbar);
    cv::createTrackbar("S Min", "Mask", &s_min, 255, on_trackbar);
    cv::createTrackbar("S Max", "Mask", &s_max, 255, on_trackbar);
    cv::createTrackbar("V Min", "Mask", &v_min, 255, on_trackbar);
    cv::createTrackbar("V Max", "Mask", &v_max, 255, on_trackbar);
    ROS_INFO("%s HRERE", TAG);

#ifdef USE_SIMULATION
    string picture_path;
    nh.param<string>("picture_path", picture_path, "");
    ROS_INFO("%s picuter path: %s", TAG, picture_path.c_str());
    cv::Mat frame = cv::imread(picture_path);
    cv::resize(frame,frame,cv::Size(img_width,img_height));
    while (ros::ok()) {
        // 捕获帧
        if (frame.empty()) {
            ROS_WARN("Empty frame received");
            break;
        }

        cv::cvtColor(frame, hsv_frame, cv::COLOR_BGR2HSV);

        on_trackbar(0, 0);
        cv::imshow("camera_node Feed", frame);

        // 使用 waitKey 控制帧率和响应键盘输入
        int key = cv::waitKey(30);
        if (key == 'q') {// 按 'q' 键退出
            break;
        }

        ros::spinOnce();// 处理 ROS 事件
    }

#else

    // 打开摄像头
    cv::VideoCapture cap(0);// 0 表示默认摄像头
    if (!cap.isOpened()) {
        ROS_ERROR("Failed to open the camera_node");
        return -1;
    } else {
        cout << "Camera opened successfully." << endl;
    }

    // 设置摄像头的分辨率为 1080p (1920x1080) 和目标帧率
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 720); // 设置分辨率宽度
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);// 设置分辨率高度
    cap.set(cv::CAP_PROP_FPS, 33);          // 设置目标帧率为 30

    cv::Mat frame;
    cout << "Starting the video feed..." << endl;// 开始视频流

    int targetFps = 20;                              // 目标帧率
    int delay = 1000 / targetFps;                    // 计算每帧的延迟时间
    auto lastTime = std::chrono::steady_clock::now();// 记录开始时间
    int frameCount = 0;                              // 统计帧数
    double currentFps = 0.0;

    while (ros::ok()) {
        // 捕获帧
        cap >> frame;
        if (frame.empty()) {
            ROS_WARN("Empty frame received");
            break;
        }

        cv::cvtColor(frame, hsv_frame, cv::COLOR_BGR2HSV);

        on_trackbar(0, 0);

        // 标注红色的区域
        // cv::Scalar lower_red1(0,100,100);

        // 计算帧率
        frameCount++;
        auto currentTime = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsed = currentTime - lastTime;
        if (elapsed.count() >= 1.0) {// 每秒更新一次帧率
            currentFps = frameCount / elapsed.count();
            frameCount = 0;
            lastTime = currentTime;
        }

        // 在帧上显示帧率
        cv::putText(frame, "FPS: " + std::to_string(static_cast<int>(currentFps)), cv::Point(10, 30),
                    cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2);

        // 显示捕获的帧
        cv::imshow("camera_node Feed", frame);

        // 使用 waitKey 控制帧率和响应键盘输入
        int key = cv::waitKey(delay);
        if (key == 'q') {// 按 'q' 键退出
            break;
        }

        ros::spinOnce();// 处理 ROS 事件
    }

    // 释放资源
    cap.release();

#endif
    cv::destroyAllWindows();
    return 0;
}
