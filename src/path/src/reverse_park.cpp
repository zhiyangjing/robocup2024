#include <iostream>
#include <path/path.h>
#include <queue>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/LaserScan.h>
#include <stack>

#define TAG " [REVERSE] "

class ReversePark : public Ability {
private:
    bool is_running_ = false;
    ros::Subscriber sub_;
    cv::Mat frame;
    int frame_height = 480;
    int frame_width = 720;
    int handle_rate_ = 20;
    int target_index = 0;  // 0 代表左侧车库，1代表右侧车库
    cv::Point target_center;
    std::vector<std::vector<cv::Point>> contours;
    std::vector<tuple<int, int, cv::Point2i>> blueContours;  // 边缘集合的下标、面积、中心点
    float lowerFraction = 0.4;
    int lowerHeight = static_cast<int>(frame_height * lowerFraction);
    int upperHeight = frame_height - lowerHeight;

public:
    ReversePark(int remain_time, ros::NodeHandle nh) : Ability(remain_time, nh) {
        string target;
        nh_.getParam("reverse_park/target", target);
        if (target == "left") {
            target_index = 0;  // 0 代表左侧车库
        } else {
            target_index = 1;  // 1代表右侧车库
        }
        ROS_INFO(TAG "Target index: %d", target_index);
    }

    void towardCenter() {
        int x = target_center.x;
        int res;
        res = -(x - frame_width / 2);
        cv::circle(frame, cv::Point(frame_width / 2, frame_height - 10), 3, cv::Scalar(0, 255, 0),
                   cv::FILLED);  // 使用 cv::FILLED 填充圆
        cv::circle(frame, cv::Point(x, frame_height - 10), 3, cv::Scalar(0, 0, 255),
                   cv::FILLED);  // 使用 cv::FILLED 填充圆
        int angle_value = max(min(res, 200), -200);
        ROS_INFO(TAG "Center x: %d,y: %d ; Angle: %d", target_center.x, target_center.y, angle_value);
        nh_.setParam("angle", angle_value);
    }

    void getContour() {
        cv::Mat hsv_frame, mask;

        cv::Rect lowerPartRect(0, frame_height - lowerHeight, frame_width, lowerHeight);
        cv::Mat lowerPart = frame(lowerPartRect);  // 提取下部分图像

        cv::cvtColor(frame, hsv_frame, cv::COLOR_BGR2HSV);

        cv::Scalar lowerBlue(100, 100, 0);
        cv::Scalar upperBlue(140, 255, 255);
        cv::inRange(hsv_frame, lowerBlue, upperBlue, mask);

        int erosion_size = 1;
        int dilation_size = 4;

        cv::Mat erosion_element =
            cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2 * erosion_size + 1, 2 * erosion_size + 1),
                                      cv::Point(erosion_size, erosion_size));

        // 定义膨胀的结构元素
        cv::Mat dilation_element =
            cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2 * dilation_size + 1, 2 * dilation_size + 1),
                                      cv::Point(dilation_size, dilation_size));

        // 腐蚀操作
        cv::erode(mask, mask, erosion_element);

        // 膨胀操作
        cv::dilate(mask, mask, dilation_element);
        // 查找轮廓
        contours.clear();
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        ROS_INFO(TAG "Contour lenght: %d", static_cast<int>(contours.size()));
    }

    void contourPreprocess() {
        blueContours.clear();

        for (int i = 0; i < contours.size(); i++) {
            auto contour = contours[i];

            // 计算矩
            cv::Moments m = cv::moments(contour);

            // 计算重心
            cv::Point2d contour_center(0, 0);
            if (m.m00 != 0) {  // 防止除以零
                contour_center = cv::Point2i(static_cast<int>(m.m10 / m.m00), static_cast<int>(m.m01 / m.m00));
            }
            blueContours.emplace_back(i, cv::contourArea(contour), contour_center);
            // double area = cv::contourArea(contour);
            // double perimeter = cv::arcLength(contour, true);
            // double diff_rate = fabs((pow((perimeter / 4), 2) - area) / area);
            // // cout << diff_rate << " " << area << " " << perimeter << endl;
            // if (diff_rate < 1) {
            //     blueContours.emplace_back(i, cv::contourArea(contour), contour_center);
            //     // 用周长和面积的关系排除掉不接近正方形的物体，测试发现这个不太准确，主要是矩形膨胀导致的
            // }
        }
        ROS_INFO(TAG "blueContours length: %d", static_cast<int>(blueContours.size()));
    }

    void getContourCenter() {
        sort(blueContours.begin(), blueContours.end(),
             [](auto const &a, auto const &b) { return get<1>(a) > get<1>(b); });

        ROS_INFO(TAG "Contour lenght: %d", static_cast<int>(blueContours.size()));
        for (int i = 0; i < blueContours.size(); i++) {
            cv::drawContours(frame, contours, get<0>(blueContours[i]), cv::Scalar(0, 0, 255), 2);
        }
        if (contours.size() >= 1) {
            for (int i = 0; i < min(2, static_cast<int>(contours.size())); i++) {
                auto color = (i == 0 ? cv::Scalar(255, 0, 0) : cv::Scalar(0, 255, 0));
                cv::drawContours(frame, contours, get<0>(blueContours[i]), color, 2);
                cv::circle(frame, get<2>(blueContours[i]), 5, color, -1);
            }
        }
        auto first_x = get<2>(blueContours[0]).y;
        auto second_x = get<2>(blueContours[1]).y;
        if (target_index == 0) {  // 左侧车库，选择x更大的那个
            target_center = (first_x > second_x) ? (get<2>(blueContours[0])) : (get<2>(blueContours[1]));
        } else {  // 右侧车库，选择x更小的那一个
            target_center = (first_x > second_x) ? (get<2>(blueContours[1])) : (get<2>(blueContours[0]));
        }
    }

    void getIntersection() {
        // 计算下部分的高度，根据给定的比例
        cv::Rect lowerPartRect(0, upperHeight, frame_width, lowerHeight);
        cv::Mat lowerPart = frame(lowerPartRect);  // 提取下部分图像

        cv::Mat hsv_frame;
        cv::cvtColor(lowerPart, hsv_frame, cv::COLOR_BGR2HSV);

        // 定义白色的HSV范围
        cv::Scalar lowerWhite(0, 0, 160);     // 白色下限
        cv::Scalar upperWhite(180, 30, 255);  // 白色上限

        // 创建白色区域的掩码
        cv::Mat mask;
        cv::inRange(hsv_frame, lowerWhite, upperWhite, mask);

        int erosion_size = 1;   // 腐蚀结构元素的大小
        int dilation_size = 1;  // 膨胀结构元素的大小

        cv::Mat element =
            cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2 * erosion_size + 1, 2 * erosion_size + 1),
                                      cv::Point(erosion_size, erosion_size));

        cv::erode(mask, mask, element);   // 腐蚀
        cv::dilate(mask, mask, element);  // 膨胀

        // 使用 Canny 边缘检测
        cv::Mat edges;
        cv::Canny(mask, edges, 50, 150, 3);
        // cv::imshow("mask",mask);

        vector<cv::Vec4i> lines_raw;  // 存储检测到的白色车道线段
        // 使用 HoughLinesP 检测线段
        cv::HoughLinesP(edges, lines_raw, 2, CV_PI / 180, 50, 20, 10);

        ROS_INFO(TAG "lines count: %d", (int) lines_raw.size());

        for (size_t i = 0; i < lines_raw.size(); i++) {
            cv::Vec4i l = lines_raw[i];
            cv::Point start(l[0], l[1] + (upperHeight));
            cv::Point end(l[2], l[3] + (upperHeight));
            cv::line(frame, start, end, cv::Scalar(0, 0, 255), 2);
        }
    }

    void imageCallback(const sensor_msgs::ImageConstPtr &msg) {
        frame = cv_bridge::toCvShare(msg, "bgr8")->image;
        cv::resize(frame, frame, cv::Size(frame_width, frame_height));
        if (frame.empty()) {
            ROS_WARN("Empty frame received");
            return;
        }

        getContour();
        getIntersection();
        contourPreprocess();
        getContourCenter();
        towardCenter();

        cv::imshow("camera_node Feed", frame);
        cv::waitKey(10);

        ros::spinOnce();  // 处理 ROS 事件
    }

    void run() {
        ROS_INFO(TAG "Reverse started to run");
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
