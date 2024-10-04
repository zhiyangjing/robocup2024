#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <string>

#define TAG "[video recorder]"

using namespace std;

class VideoRecorder {
private:
    cv::VideoCapture cap_;
    ros::NodeHandle nh_;
    cv::VideoWriter video;
    int fps = 40;
    string output_path_;

public:
    VideoRecorder(ros::NodeHandle &nh) : nh_(nh) {
        cap_.open(0);// 打开摄像头
        nh_.param<string>("output_video_path", output_path_, "");
        int codec = cv::VideoWriter::fourcc('M', 'J', 'P', 'G');
        int frame_width = static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_WIDTH));
        int frame_height = static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_HEIGHT));
        cv::Size frame_size(frame_width, frame_width);
        video = cv::VideoWriter(output_path_, codec, fps, frame_size, true);
        if (!cap_.isOpened()) {
            ROS_ERROR("Failed to open the camera");
            ros::shutdown();
        }
    }

    void record() {
        cv::Mat frame;
        cap_ >> frame;// 捕获图像
        if (frame.empty()) {
            ROS_WARN("Empty frame received");
            return;
        }
        video.write(frame);
    }
    ~VideoRecorder() { cap_.release(); }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "video recorder");
    ros::NodeHandle nh;
    VideoRecorder video_recorder(nh);
    ROS_INFO("%s Camera node init...", TAG);
    ros::Rate rate(40);
    while (ros::ok()) {
        video_recorder.record();
        ros::spinOnce();
        rate.sleep();
    }

    ros::spin();
    return 0;
}
