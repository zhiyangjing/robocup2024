#include <spdlog/spdlog.h>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>

int main(int argc,char** argv) {
    ros::init(argc,argv,"camera_node");

    ros::NodeHandle nh;

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
        cv::imshow("camera_node Feed" , frame);
        if (cv::waitKey(30) >= 0) break;
        ros::spinOnce();
    }
    cap.release();
    cv::destroyAllWindows();
    return 0;
}



