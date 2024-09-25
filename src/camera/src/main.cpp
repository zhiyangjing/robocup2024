#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

class ImagePublisher {
public:
    ImagePublisher(ros::NodeHandle &nh) {
        pub_ = nh.advertise<sensor_msgs::Image>("image_topic", 1);
        cap_.open(0);// 打开摄像头
        if (!cap_.isOpened()) {
            ROS_ERROR("Failed to open the camera");
            ros::shutdown();
        }
    }

    void publishImage() {
        cv::Mat frame;
        cap_ >> frame;// 捕获图像
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
        pub_.publish(msg);
    }
    ~ImagePublisher() { cap_.release(); }

private:
    ros::Publisher pub_;
    cv::VideoCapture cap_;
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "camera_node");
    ros::NodeHandle nh;
    ImagePublisher imagePublisher(nh);

    ros::Rate rate(20);
    while (ros::ok()) {
        imagePublisher.publishImage();
        ros::spinOnce();
        rate.sleep();
    }

    ros::spin();
    return 0;
}
