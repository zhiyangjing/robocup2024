#include <iostream>
#include <path/path.h>
#include <queue>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/LaserScan.h>
#include <stack>

#define TAG " [REVERSE]"

class ReversePark : public Ability {
private:
    bool is_running_ = false;
    ros::Subscriber sub_;
    cv::Mat frame;
    int frame_height = 480;
    int frame_width = 720;
    int handle_rate_ = 20;
    std::vector<tuple<int, int, cv::Point2i>> sorted_contours;  // 边缘集合的下标、面积、中心点

public:
    ReversePark(int remain_time, ros::NodeHandle nh) : Ability(remain_time, nh) {}

    void getCenter() {
        cv::Mat hsv, mask;
        cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
        cv::Scalar lower_blue(100, 50, 0);
        cv::Scalar upper_blue(140, 255, 255);
        cv::inRange(hsv, lower_blue, upper_blue, mask);

        // 查找轮廓
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        sorted_contours.clear();
        for (int i = 0; i < contours.size(); i++) {
            auto contour = contours[i];

            // 计算矩
            cv::Moments m = cv::moments(contour);

            // 计算重心
            cv::Point2d center(0, 0);
            if (m.m00 != 0) {  // 防止除以零
                center = cv::Point2i(static_cast<int>(m.m10 / m.m00), static_cast<int>(m.m01 / m.m00));
            }
            sorted_contours.emplace_back(i, cv::contourArea(contour), center);
        }

        sort(sorted_contours.begin(), sorted_contours.end(), [](auto const &a, auto const &b) {
            return get<1>(a) > get<1>(b);
        });

        ROS_INFO(TAG "Contour lenght: %d", (int) sorted_contours.size());
        // cv::drawContours(frame, contours, -1, cv::Scalar(0, 0, 255), 2);
        if (contours.size() >= 1) {
            for (int i = 0; i < min(2, static_cast<int>(contours.size())); i++) {
                auto color = (i == 0 ? cv::Scalar(255, 0, 0) : cv::Scalar(0, 0, 255));
                cv::drawContours(frame, contours, get<0>(sorted_contours[i]), color, 2);
                cv::circle(frame, get<2>(sorted_contours[i]), 5, color, -1);
            }
        }
    }

    void imageCallback(const sensor_msgs::ImageConstPtr &msg) {
        frame = cv_bridge::toCvShare(msg, "bgr8")->image;
        cv::resize(frame, frame, cv::Size(frame_width, frame_height));
        if (frame.empty()) {
            ROS_WARN("Empty frame received");
            return;
        }

        getCenter();

        cv::imshow("camera_node Feed", frame);
        cv::waitKey(10);

        ros::spinOnce();  // 处理 ROS 事件
    }

    void run() {
        ROS_INFO(TAG "TraceLine started to run");
        is_running_ = true;
        sub_ = nh_.subscribe("/image_topic/back", 1, &ReversePark::imageCallback, this);

        ros::Rate handle_rate(handle_rate_);  // 处理频率
        while (ros::ok() && is_running_) {
            ros::spinOnce();
            handle_rate.sleep();
        }
        sub_.shutdown();
    }

    void stop() { is_running_ = false; }
};

/**
 * @brief 这里只是一个测试用的main函数，实际上真正使用这里的功能是在main.cpp
 *
 */
int main(int argc, char **argv) {
    // 测试节点
    ros::init(argc, argv, "trace_line");
    ros::NodeHandle nh;
    cv::namedWindow("camera_node Feed", cv::WINDOW_AUTOSIZE);

    ReversePark reverse_park(10, nh);
    reverse_park.run();
    cv::destroyAllWindows();
    return 0;
}
