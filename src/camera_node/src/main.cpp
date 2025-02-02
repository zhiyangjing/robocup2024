#include <common_utils/common.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <string>

#define TAG " [camera node] "

using namespace std;

/** \addtogroup camera
 * @{
 */
class CameraNode {
private:
    ros::Publisher pub_;
    cv::VideoCapture cap_;
    string video_file_path_;

public:
    CameraNode(ros::NodeHandle &nh) {
        string node_name = ros::this_node::getName();
        ROS_INFO(TAG "%s initing...", node_name.c_str());
        pub_ = nh.advertise<sensor_msgs::Image>("/image_topic", 1);
#ifdef USE_SIMULATION
        ROS_INFO(TAG " Now using simulation");
        nh.param<string>("camera_node/video_path", video_file_path_, "");
        ROS_INFO(TAG "video path: %s", video_file_path_.c_str());
        cap_.open(video_file_path_);
#else
        ROS_INFO(TAG "Now using real camera");
        int camera_port = 0;
        nh.getParam("camera_node/camera_port", camera_port);
        ROS_INFO(TAG COLOR_GREEN "Node now using port: %d" COLOR_RESET, camera_port);
        cap_.open("/dev/camera" + to_string(camera_port), cv::CAP_V4L2);  // 打开摄像头
        cap_.set(cv::CAP_PROP_FRAME_WIDTH, 640);                          // 设置宽度为640
        cap_.set(cv::CAP_PROP_FRAME_HEIGHT, 480);                         // 设置高度为480
        // cap_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
        ROS_INFO(TAG " Camera open succeeded");
#endif
        if (!cap_.isOpened()) {
            ROS_ERROR("Failed to open the camera");
            ros::shutdown();
        }
    }

    void publishImage() {
        cv::Mat frame;
        cap_ >> frame;  // 捕获图像
#ifdef USE_SIMULATION
        // 如果从mp4视频中读取，需要翻转一下
        // cv::flip(frame, frame, -1);
#endif
        if (frame.empty()) {
            ROS_WARN("Empty frame received");
            return;
        }
        // 确保 frame 是连续的
        if (!frame.isContinuous()) {
            ROS_ERROR("Frame is not continuous");
            return;
        }

        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
        ROS_INFO("Image Published");
        pub_.publish(msg);
    }
    ~CameraNode() { cap_.release(); }
};
/**
 * @}
 * 
 */


int main(int argc, char **argv) {
    ros::init(argc, argv, "camera_node");
    ros::NodeHandle nh;
    ROS_INFO("%s Camera node init...", TAG);
    CameraNode imagePublisher(nh);
    int frame_rate = 10;
    nh.getParam("camera_node/frame_rate", frame_rate);
    ros::Rate rate(frame_rate);
    while (ros::ok()) {
        imagePublisher.publishImage();
        ros::spinOnce();
        rate.sleep();
    }

    ros::spin();
    return 0;
}
