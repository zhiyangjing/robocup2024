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
    int fps = 30;
    string output_path_;
    int total_frame{};

public:
    VideoRecorder(ros::NodeHandle &nh) : nh_(nh) {
        cap_.open(0);// 打开摄像头
        nh_.param<string>("output_video_path", output_path_, "");
        ROS_INFO("%s video path: %s", TAG, output_path_.c_str());
        int codec = cv::VideoWriter::fourcc('M', 'J', 'P', 'G');
        int frame_width = static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_WIDTH));
        int frame_height = static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_HEIGHT));
        int fps = cap_.get(cv::CAP_PROP_FPS);
        ROS_INFO("%s capture frame rate: %d ", TAG, fps);
        cv::Size frame_size(frame_width, frame_height);
        video = cv::VideoWriter(output_path_, codec, fps, frame_size, true);
        if (!cap_.isOpened()) {
            ROS_ERROR("Failed to open the camera");
            ros::shutdown();
        }
        if (!video.isOpened()) {
            ROS_ERROR("Failed to create video");
            ros::shutdown();
        }
    }

    void run() {
        ROS_INFO("%s started runing ! ", TAG);
        cv::Mat frame;
        while (ros::ok()) {
            cap_ >> frame;// 捕获图像
            if (frame.empty()) {
                ROS_WARN("Empty frame received");
                return;
            }
            video.write(frame);
            total_frame += 1;
            ROS_INFO("%s current frame: %d", TAG, total_frame);
            // cv::imshow("video writer", frame);
            // auto key = cv::waitKey(1);
            // if (key == 'q') {
            //     ROS_INFO("%s Exting video recorder! ", TAG);
            //     ros::shutdown();
            //     break;
            // }
            ros::spinOnce();
        }
    }
    ~VideoRecorder() {
        cap_.release();
        video.release();
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "video recorder");
    cv::namedWindow("video writer", cv::WINDOW_AUTOSIZE);
    ros::NodeHandle nh;
    VideoRecorder video_recorder(nh);
    ROS_INFO("%s Video Recorder node init...", TAG);
    video_recorder.run();
    cv::destroyAllWindows();
    return 0;
}
