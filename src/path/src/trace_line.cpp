#include <chrono>
#include <common_utils/common.h>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

using namespace std;

class TraceLine {
private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;

    int frameCount = 0;
    std::chrono::steady_clock::time_point lastTime = std::chrono::steady_clock::now();
    int frame_height = 480;
    int frame_width = 720;
    int line_pos = frame_width * 0.422;
    double currentFps = 0;


public:
    TraceLine(ros::NodeHandle &nh) : nh_(nh) {
        sub_ = nh_.subscribe("image_topic", 1, &TraceLine::imageCallback, this);
    }

    // 计算并显示帧率的函数
    void imageCallback(const sensor_msgs::ImageConstPtr &msg) {
        try {
            cv::Mat frame = cv_bridge::toCvShare(msg, "bgr8")->image;

            cv::resize(frame, frame, cv::Size(frame_width, frame_height));
            if (frame.empty()) {
                ROS_WARN("Empty frame received");
                return;
            }

            cv::imshow("camera_node Feed", frame);
            cv::waitKey(30);

            ros::spinOnce();// 处理 ROS 事件
        } catch (cv_bridge::Exception & e) {
            ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
        }
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "trace_line");
    ros::NodeHandle nh;
    cv::namedWindow("camera_node Feed", cv::WINDOW_AUTOSIZE);    

    TraceLine lightProcessor(nh);

    ros::spin();
    cv::destroyAllWindows();
    return 0;
}