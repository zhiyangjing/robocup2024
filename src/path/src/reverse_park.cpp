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
    float offset_top_ratio = 0.1;
    float offset_bottom_ratio = 0.1;
    int offset_top = static_cast<int>(offset_top_ratio * frame_height);
    int offset_bottom = static_cast<int>(offset_bottom_ratio * frame_height);
    cv::Point target_center;
    std::vector<std::vector<cv::Point>> contours;
    std::vector<tuple<int, int, cv::Point2i>> blueContours;  // 边缘集合的下标、面积、中心点
    float lowerFraction = 0.55;
    int lowerHeight = static_cast<int>(frame_height * lowerFraction);
    int upperHeight = frame_height - lowerHeight;
    vector<cv::Vec4i> lines_raw;                                       // 存储检测到的白色车道线段
    vector<tuple<cv::Vec4i, float, float, cv::Vec2i, int>> laneLines;  // 线段，长度，斜率 , 中点， 和下边界的交点
    Buffer<int> right_point;
    Buffer<int> left_point;

public:
    ReversePark(int remain_time, ros::NodeHandle nh) : Ability(remain_time, nh) {
        right_point = Buffer<int>(5);
        left_point = Buffer<int>(5);
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
        res = -(x - frame_width / 2) * 4;
        cv::circle(frame, cv::Point(frame_width / 2, frame_height - 10), 3, cv::Scalar(0, 255, 0),
                   cv::FILLED);  // 使用 cv::FILLED 填充圆
        cv::circle(frame, cv::Point(x, frame_height - 10), 3, cv::Scalar(0, 0, 255),
                   cv::FILLED);  // 使用 cv::FILLED 填充圆
        int angle_value = max(min(res, 200), -200);
        // ROS_INFO(TAG "Center x: %d,y: %d ; Angle: %d", target_center.x, target_center.y, angle_value);
        nh_.setParam("angle", angle_value);
    }

    void getContour() {
        if (frame.empty()) {
            ROS_WARN(TAG "getContour frame empty");
        }
        cv::Mat hsv_frame, mask;

        cv::Rect midPartRect(0, offset_top, frame_width, frame_height - offset_bottom - offset_top);

        cv::Mat midPart = frame(midPartRect);  // 提取下部分图像

        cv::cvtColor(midPart, hsv_frame, cv::COLOR_BGR2HSV);

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
        // ROS_INFO(TAG "Contour lenght: %d", static_cast<int>(contours.size()));
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
        // ROS_INFO(TAG "blueContours length: %d", static_cast<int>(blueContours.size()));
    }

    float calculateSlope(const cv::Vec4i &line) {
        float dx = line[2] - line[0];
        float dy = line[3] - line[1];
        if (dx == 0) {
            return numeric_limits<float>::infinity();  // 处理垂直线
        }
        return dy / dx;  // 计算斜率
    }

    void linePreprocess() {
        if (lines_raw.empty()) {
            ROS_WARN(TAG "No line to preprocess");
        } else {
            laneLines.clear();
            // ROS_INFO(TAG "lines count %d", (int) lines_raw.size());
            for (const auto &line : lines_raw) {
                float slope = calculateSlope(line);

                if (fabs(slope) < 0.7 or fabs(slope) > 20) {
                    continue;  // 去除横向线段，以及过于垂直的线段
                }

                int x0 = line[0], y0 = line[1], x1 = line[2], y1 = line[3];
                int intersection_pos = (lowerHeight - y0) * (x1 - x0) / (y1 - y0) + x0;

                double lineLength = cv::norm(cv::Point(line[0], line[1]) - cv::Point(line[2], line[3]));
                laneLines.emplace_back(line, lineLength, slope,
                                       cv::Vec2i((line[0] + line[2]) / 2, (line[1] + line[3]) / 2 + upperHeight),
                                       intersection_pos);

                cv::Point start(line[0], line[1] + (upperHeight));
                cv::Point end(line[2], line[3] + (upperHeight));
                cv::line(frame, start, end, cv::Scalar(0, 0, 255), 2);
                cv::circle(frame, cv::Point(intersection_pos, frame_height - 5), 3, cv::Scalar(255, 0, 255), -1);
            }
        }
        // ROS_INFO(TAG "lines count: %d", (int) laneLines.size());
    }

    void getIntersection() {
        int target_x = target_center.x;
        sort(laneLines.begin(), laneLines.end(), [target_x](auto const &a, auto const &b) {
            return abs(get<3>(a)[0] - target_x) < abs(get<3>(b)[0] - target_x);
        });
        bool right_lane_found = false;
        bool left_lane_found = false;
        for (auto &Lane : laneLines) {
            auto line = get<0>(Lane);
            auto length = get<1>(Lane);
            auto slope = get<2>(Lane);
            auto center_x = get<3>(Lane)[0];
            if (not right_lane_found and slope > 0 and center_x > target_x) {
                cv::Point start(line[0], line[1] + (upperHeight));
                cv::Point end(line[2], line[3] + (upperHeight));
                cv::line(frame, start, end, cv::Scalar(255, 0, 0), 2);
                right_lane_found = true;
                right_point.push(get<4>(Lane));
                ROS_INFO(TAG "RIGHT: %d", right_point.avg());
            } else if (not left_lane_found and length > 50 and slope < 0 and center_x < target_x) {
                cv::Point start(line[0], line[1] + (upperHeight));
                cv::Point end(line[2], line[3] + (upperHeight));
                cv::line(frame, start, end, cv::Scalar(0, 255, 0), 2);
                left_lane_found = true;
                left_point.push(get<4>(Lane));
                ROS_INFO(TAG "LEFT: %d", left_point.avg());
            }
        }
    }

    void getContourCenter() {
        sort(blueContours.begin(), blueContours.end(),
             [](auto const &a, auto const &b) { return get<1>(a) > get<1>(b); });

        // ROS_INFO(TAG "Contour lenght: %d", static_cast<int>(blueContours.size()));

        // ****************************************可视化****************************************
        for (size_t i = 0; i < contours.size(); i++) {
            cv::Scalar greenColor(0, 255, 0);  // 绿色
            // 添加 Y 轴偏移
            for (size_t j = 0; j < contours[i].size(); j++) {
                contours[i][j].y += offset_top;
            }
        }

        for (int i = 0; i < blueContours.size(); i++) {
            cv::drawContours(frame, contours, get<0>(blueContours[i]), cv::Scalar(0, 0, 255), 2);
        }
        // **************************************** ****************************************

        bool valid = true;
        if (blueContours.size() >= 2) {
            auto first_size = get<1>(blueContours[0]);
            auto second_size = get<1>(blueContours[1]);
            auto first_x = get<2>(blueContours[0]).x;
            auto second_x = get<2>(blueContours[1]).x;

            float ratio = static_cast<float>(first_size) / second_size;
            if (ratio > 5 or second_x < 100 or second_x > 620) {
                // 如果大小差异过大，则说明小一点的那个不可能是二号标志牌，这里的valid最终会是的如果有第二个蓝色区域，那么该蓝色区域会被判否。
                // 由于这里的判断不太好和下面的if段匹配，所以分开来写。
                valid = false;
                cout << COLOR_RED "not valid: " << COLOR_RESET << ratio << endl;
            }
        }
        if (blueContours.size() >= 2 and valid) {
            auto first_x = get<2>(blueContours[0]).x;
            auto second_x = get<2>(blueContours[1]).x;
            if (target_index == 0) {  // 左侧车库，选择x更大的那个
                target_center = (first_x > second_x) ? (get<2>(blueContours[0])) : (get<2>(blueContours[1]));
            } else {  // 右侧车库，选择x更小的那一个
                target_center = (first_x > second_x) ? (get<2>(blueContours[1])) : (get<2>(blueContours[0]));
            }
            // ********************************可视化************************************************
            {
                cv::drawContours(frame, contours, get<0>(blueContours[0]), cv::Scalar(255, 0, 0), 2);
                auto [x, y] = get<2>(blueContours[0]);
                cv::circle(frame, cv::Point(x, y + offset_top), 5, cv::Scalar(255, 0, 0), -1);
            }
            {
                cv::drawContours(frame, contours, get<0>(blueContours[1]), cv::Scalar(0, 255, 0), 2);
                auto [x, y] = get<2>(blueContours[1]);
                cv::circle(frame, cv::Point(x, y + offset_top), 5, cv::Scalar(0, 255, 0), -1);
            }

            // ***************************************************************************************
        } else if (blueContours.size() >= 1) {
            target_center = get<2>(blueContours[0]);
            {
                cv::drawContours(frame, contours, get<0>(blueContours[0]), cv::Scalar(0, 255, 0), 2);
                auto [x, y] = get<2>(blueContours[0]);
                cv::circle(frame, cv::Point(x, y + offset_top), 5, cv::Scalar(0, 255, 0), -1);
            }
        } else {
            target_center = cv::Point(frame_width / 2, frame_height / 2);
        }
    }

    void getLines() {
        // 计算下部分的高度，根据给定的比例
        cv::Rect lowerPartRect(0, upperHeight, frame_width, lowerHeight);
        cv::Mat lowerPart = frame(lowerPartRect);  // 提取下部分图像

        cv::Mat hsv_frame;
        cv::cvtColor(lowerPart, hsv_frame, cv::COLOR_BGR2HSV);

        // 定义白色的HSV范围
        cv::Scalar lowerWhite(0, 0, 160);     // 白色下限
        cv::Scalar upperWhite(180, 35, 255);  // 白色上限

        // 创建白色区域的掩码
        cv::Mat mask;
        cv::inRange(hsv_frame, lowerWhite, upperWhite, mask);
        // cv::imshow("white", mask);

        int erosion_size = 1;   // 腐蚀结构元素的大小
        int dilation_size = 1;  // 膨胀结构元素的大小

        cv::Mat element =
            cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2 * erosion_size + 1, 2 * erosion_size + 1),
                                      cv::Point(erosion_size, erosion_size));

        cv::erode(mask, mask, element);   // 腐蚀
        cv::dilate(mask, mask, element);  // 膨胀
        cv::imshow("iamge", mask);

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        cv::Mat contourImage = cv::Mat::zeros(mask.size(), CV_8UC1);

        for (int i = 0; i < contours.size(); i++) {
            auto area = cv::contourArea(contours[i]);
            if (area > 200) {
                cv::drawContours(contourImage, contours, i, cv::Scalar(255), 1);  // 绘制白色的轮廓线，宽度为1像素
            }
        }

        // // 使用 Canny 边缘检测
        // cv::Mat edges;
        // cv::Canny(mask, edges, 50, 200, 3);
        // // cv::imshow("mask",mask);

        cv::imshow("contour", contourImage);

        // 使用 HoughLinesP 检测线段
        cv::HoughLinesP(contourImage, lines_raw, 2, CV_PI / 180, 30, 25, 20);
        // 在获得中点之后再搜索可用线段。
    }

    void imageCallback(const sensor_msgs::ImageConstPtr &msg) {
        frame = cv_bridge::toCvShare(msg, "bgr8")->image;
        cv::resize(frame, frame, cv::Size(frame_width, frame_height));
        if (frame.empty()) {
            ROS_WARN("Empty frame received");
            return;
        }

        // 流水线处理，顺序调整得检查依赖关系
        getContour();
        contourPreprocess();
        getContourCenter();
        // 在获得中点之后再搜索可用线段。
        getLines();
        linePreprocess();
        getIntersection();
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
