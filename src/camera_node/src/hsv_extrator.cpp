#include <chrono>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#define TAG "[hsv_extrator]"
using namespace std;

// cv::Scalar lowerBlue(100, 100, 0);
// cv::Scalar upperBlue(140, 255, 255);

cv::Mat frame;
int h_min = 100, s_min = 100, v_min = 0;
int h_max = 140, s_max = 255, v_max = 255;
cv::Mat hsv_frame, mask;  // 全局变量，存储图像和处理结果
int img_height = 480;
int img_width = 720;
int erosion_size = 1;   // 腐蚀结构元素的大小
int dilation_size = 1;  // 膨胀结构元素的大小

void on_trackbar(int, void *) {
    cv::inRange(hsv_frame, cv::Scalar(h_min, s_min, v_min), cv::Scalar(h_max, s_max, v_max), mask);

    // 定义腐蚀的结构元素
    cv::Mat erosion_element = cv::getStructuringElement(
        cv::MORPH_RECT, cv::Size(2 * erosion_size + 1, 2 * erosion_size + 1), cv::Point(erosion_size, erosion_size));

    // 定义膨胀的结构元素
    cv::Mat dilation_element =
        cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2 * dilation_size + 1, 2 * dilation_size + 1),
                                  cv::Point(dilation_size, dilation_size));

    // 腐蚀操作
    cv::erode(mask, mask, erosion_element);

    // 膨胀操作
    cv::dilate(mask, mask, dilation_element);

    cv::imshow("Mask", mask);  // 显示二值化的掩码

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    ROS_INFO(TAG "contours length: %d", static_cast<int>(contours.size()));

    if (contours.size() < 100) {
        for (size_t i = 0; i < contours.size(); i++) {
            cv::Scalar greenColor(0, 255, 0);  // 绿色
            cv::drawContours(frame, contours, static_cast<int>(i), greenColor, 2, cv::LINE_8);
        }
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "hsv_extrator");
    ros::NodeHandle nh;

    // 创建窗口和滑动条
    cv::namedWindow("Camera Feed", cv::WINDOW_AUTOSIZE);
    cv::namedWindow("Mask", cv::WINDOW_AUTOSIZE);

    // 创建滑动条，调节 HSV 值
    cv::createTrackbar("erosion", "Mask", &erosion_size, 10, on_trackbar);
    cv::createTrackbar("dilation", "Mask", &dilation_size, 10, on_trackbar);
    // cv::createTrackbar("H Min", "Mask", &h_min, 179, on_trackbar);
    // cv::createTrackbar("H Max", "Mask", &h_max, 179, on_trackbar);
    // cv::createTrackbar("S Min", "Mask", &s_min, 255, on_trackbar);
    // cv::createTrackbar("S Max", "Mask", &s_max, 255, on_trackbar);
    // cv::createTrackbar("V Min", "Mask", &v_min, 255, on_trackbar);
    // cv::createTrackbar("V Max", "Mask", &v_max, 255, on_trackbar);
    ROS_INFO("%s HRERE", TAG);

#ifdef USE_SIMULATION
    string source_path;
    nh.param<string>("source_path", source_path, "");
    ROS_INFO("%s picuter path: %s", TAG, source_path.c_str());
    std::string extension = source_path.substr(source_path.find_last_of(".") + 1);
    std::transform(extension.begin(), extension.end(), extension.begin(), ::tolower);  // 转为小写
    cv::VideoCapture cap;
    bool is_video = false;
    if (extension == "png" || extension == "jpg" || extension == "jpeg") {
        // 处理图片
        frame = cv::imread(source_path);
        cv::resize(frame, frame, cv::Size(img_width, img_height));
    } else if (extension == "mp4" || extension == "avi") {
        cap.open(source_path);
        is_video = true;
    } else {
        ROS_WARN("Unsupported file type: %s", source_path.c_str());
    }

    int frame_rate = 5;
    nh.getParam("frame_rate", frame_rate);
    ros::Rate looprate(frame_rate);
    while (ros::ok()) {
        // 捕获帧
        if (is_video) {
            cap >> frame;
        }
        if (frame.empty()) {
            ROS_WARN("Empty Frame received");
            continue;
        }
        cv::resize(frame, frame, cv::Size(img_width, img_height));
        if (frame.empty()) {
            ROS_WARN("Empty frame received");
            break;
        }

        cv::cvtColor(frame, hsv_frame, cv::COLOR_BGR2HSV);

        on_trackbar(0, 0);

        cv::imshow("camera_node Feed", frame);

        // 使用 waitKey 控制帧率和响应键盘输入
        int key = cv::waitKey(10);
        if (key == 'q') {  // 按 'q' 键退出
            break;
        }

        looprate.sleep();
        ros::spinOnce();  // 处理 ROS 事件
    }

#else

    // 打开摄像头
    cv::VideoCapture cap(0);  // 0 表示默认摄像头
    if (!cap.isOpened()) {
        ROS_ERROR("Failed to open the camera_node");
        return -1;
    } else {
        cout << "Camera opened successfully." << endl;
    }

    cout << "Starting the video feed..." << endl;  // 开始视频流

    int targetFps = 20;                                // 目标帧率
    int delay = 1000 / targetFps;                      // 计算每帧的延迟时间
    auto lastTime = std::chrono::steady_clock::now();  // 记录开始时间
    int frameCount = 0;                                // 统计帧数
    double currentFps = 0.0;
    ros::Rate loop_rate(5);

    while (ros::ok()) {
        // 捕获帧
        cap >> frame;
        if (frame.empty()) {
            ROS_WARN("Empty frame received");
            loop_rate.sleep();
            continue;
        }
        cv::resize(frame, frame, cv::Size(img_width, img_height));

        cv::cvtColor(frame, hsv_frame, cv::COLOR_BGR2HSV);

        on_trackbar(0, 0);

        // 标注红色的区域
        // cv::Scalar lower_red1(0,100,100);

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
        cv::putText(frame, "FPS: " + std::to_string(static_cast<int>(currentFps)), cv::Point(10, 30),
                    cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2);

        // 显示捕获的帧
        cv::imshow("camera_node Feed", frame);

        // 使用 waitKey 控制帧率和响应键盘输入
        int key = cv::waitKey(delay);
        if (key == 'q') {  // 按 'q' 键退出
            break;
        }

        ros::spinOnce();  // 处理 ROS 事件
        loop_rate.sleep();
    }

    // 释放资源
    cap.release();

#endif
    cv::destroyAllWindows();
    return 0;
}
