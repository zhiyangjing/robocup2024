#include <chrono>
#include <common_utils/common.h>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <path/path.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/LaserScan.h>

#define TAG "[Trace Line]"

using namespace std;

void detectWhite(const cv::Mat &frame) {
    ROS_INFO(COLOR_RED TAG COLOR_RESET " detect white ");
    // 将输入的 BGR 图像转为 HSV（色相、饱和度、亮度）颜色空间
    cv::Mat hsvFrame;
    cv::cvtColor(frame, hsvFrame, cv::COLOR_BGR2HSV);

    // 定义白色的阈值范围
    // 白色的 HSV 值接近 (0, 0, 255)，因此饱和度 (S) 要低，亮度 (V) 要高
    cv::Scalar lowerWhite(0, 0, 160);     // 最低值 (0, 0, 200)
    cv::Scalar upperWhite(180, 30, 255);  // 最高值 (180, 55, 255)

    // 创建白色部分的遮罩
    cv::Mat mask;
    cv::inRange(hsvFrame, lowerWhite, upperWhite, mask);

    // 查找遮罩中的轮廓
    vector<vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    // 在原始帧上用红色绘制轮廓
    cv::drawContours(frame, contours, -1, cv::Scalar(0, 0, 255), 2);  // 红色，线宽2
}

float calculateSlope(const cv::Vec4i &line) {
    float dx = line[2] - line[0];
    float dy = line[3] - line[1];
    if (dx == 0) {
        return numeric_limits<float>::infinity();  // 处理垂直线
    }
    return dy / dx;  // 计算斜率
}

/**
 * @brief 这是一个辅助函数，用于可视化图像
 * @details 标注出识别到的白色区域，同时用红色标注出识别到的线条以及斜率
 * 
 * 
 * @param image 
 */
void visualizeLineInfo(cv::Mat image) {
    // 检查输入图像是否为空
    if (image.empty()) {
        cerr << "Error: Input image is empty!" << endl;
        return;
    }
    float lowerFraction = 0.4;
    // 获取图像的高度和宽度
    int height = image.rows;
    int width = image.cols;

    // 计算下部分的高度，根据给定的比例
    int lowerHeight = static_cast<int>(height * lowerFraction);
    cv::Rect lowerPartRect(0, height - lowerHeight, width, lowerHeight);
    cv::Mat lowerPart = image(lowerPartRect);  // 提取下部分图像

    // 转换为HSV颜色空间
    cv::Mat hsv;
    cv::cvtColor(lowerPart, hsv, cv::COLOR_BGR2HSV);

    // 定义白色的HSV范围
    cv::Scalar lowerWhite(0, 0, 160);     // 白色下限
    cv::Scalar upperWhite(180, 30, 255);  // 白色上限

    // 创建白色区域的掩码
    cv::Mat mask;
    cv::inRange(hsv, lowerWhite, upperWhite, mask);

    // 形态学操作 - 腐蚀和膨胀
    int erosion_size = 1;   // 腐蚀结构元素的大小
    int dilation_size = 1;  // 膨胀结构元素的大小

    cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2 * erosion_size + 1, 2 * erosion_size + 1),
                                                cv::Point(erosion_size, erosion_size));

    cv::erode(mask, mask, element);   // 腐蚀
    cv::dilate(mask, mask, element);  // 膨胀

    // 使用 Canny 边缘检测
    cv::Mat edges;
    cv::Canny(mask, edges, 50, 150, 3);
    // cv::imshow("canny", edges);

    // 存储检测到的线段
    vector<cv::Vec4i> lines;
    // 使用 HoughLinesP 检测线段
    cv::HoughLinesP(edges, lines, 2, CV_PI / 180, 50, 30, 10);

    // 在下半部分的原始图像上绘制绿色轮廓
    vector<vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    for (size_t i = 0; i < contours.size(); i++) {
        cv::Scalar greenColor(0, 255, 0);  // 绿色
        for (size_t j = 0; j < contours[i].size(); j++) {
            contours[i][j].y += height - lowerHeight;  // Y 轴偏移
        }
        cv::drawContours(image, contours, static_cast<int>(i), greenColor, 2, cv::LINE_8);
    }

    // 在原始图像上绘制检测到的线段并显示斜率
    for (size_t i = 0; i < lines.size(); i++) {
        cv::Vec4i l = lines[i];
        cv::Point start(l[0], l[1] + (height - lowerHeight));
        cv::Point end(l[2], l[3] + (height - lowerHeight));
        cv::line(image, start, end, cv::Scalar(0, 0, 255), 2);

        float slope = calculateSlope(l);
        string slopeText = "Slope: " + to_string(slope);
        cv::Point midPoint((start.x + end.x) / 2, (start.y + end.y) / 2);
        cv::putText(image, slopeText, midPoint, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
    }

    // cv::imshow("detect line result", image);
}

TraceLine::TraceLine(int remain_time, ros::NodeHandle &nh) : Ability(remain_time, nh) {
    prev_neg_slope = Buffer<float>(5);
    prev_pos_slope = Buffer<float>(5);
    prev_angle = Buffer<int>(3);
    prev_center = Buffer<int>(6);
    blue_horizontal_times = Buffer<float>(4, 0);

    // 控制点个数
    int point_nums = 19;

    VectorXf weights(3);
    VectorXf x(point_nums);
    VectorXf y(point_nums);
    VectorXf c(point_nums);
    VectorXf z(point_nums);
    weights << 1, 1, 2;  // 斜率的权重是1，中心点的权重是1.5，weight的实现有点问题，总之越大越不重要

    // 针对中心的直行优化
    x<<-0.5,-0.776,-0.4,-0.46,-0.584,-0.70,-0.51,-0.48,-1.397,0,0,-1.25,0,-0.6,0,0,-0.6,-0.41,-0.58;
    y<<0.620,0.43,0.84,0.753,0.575,0.496,0.51,0.578,0,1.4,1.2,0,0.67,0.5,0.52,0.63,0,0,0;
    c<<300,460,206,247,350,400,471,217,450,246,184,505,215,310,310,240,497,343,310;
    z<<0,0,0,0,0,0,150,-150,120,-120,-200,200,-200,0,0,-150,200,150,0;


    // x << -0.5, -0.776, -0.4, -0.46, -0.584, -0.70, -0.51, -0.48, -1.397, 0, 0, -1.25, 0, -0.6, 0, 0, -0.6, -0.41;
    // y << 0.620, 0.43, 0.84, 0.753, 0.575, 0.496, 0.51, 0.578, 0, 1.4, 1.2, 0, 0.67, 0.5, 0.52, 0.63, 0, 0;
    // c << 300, 460, 206, 247, 350, 400, 471, 217, 450, 246, 184, 505, 215, 310, 310, 240, 497, 343;
    // z << 0, 0, 0, 0, 0, 0, 150, -150, 120, -120, -200, 200, -200, 0, 0, -150, 200, 150;

    // x << -0.5, -0.776, -0.4, -0.46, -0.584, -0.70, -0.51, -0.48, -1.397, 0, 0, -1.25, 0, -0.6, 0, 0, -0.6;
    // y << 0.620, 0.43, 0.84, 0.753, 0.575, 0.496, 0.51, 0.578, 0, 1.4, 1.2, 0, 0.67, 0.5, 0.52, 0.63, 0;
    // c << 300, 460, 206, 247, 350, 400, 471, 217, 450, 246, 184, 505, 215, 310, 310, 240, 497;
    // z << 0, 0, 0, 0, 0, 0, 150, -150, 120, -120, -200, 200, -200, 0, 0, -150, 200;

    // x << -0.5,-0.776,-0.4,-0.46,-0.584,-0.70,-0.51,-0.48,-1.397,0,0,-1.25,0,-0.6,0;
    // y << 0.620,0.43,0.84,0.753,0.575,0.496,0.51,0.578,0,1.4,1.2,0,0.67,0.5,0.52;
    // c << 300,460,206,247,350,400,471,217,450,246,184,505,215,310,310;
    // z << 0,0,0,0,0,0,100,-100,150,-150,-200,200,-200,0,0;

    // x << -0.5, -0.776, -0.4, -0.46, -0.584, -0.70, -0.51, -0.48, -1.397, 0, 0, -1.25, -0.447, -0.408, 0, 0;
    // y << 0.620, 0.43, 0.84, 0.753, 0.575, 0.496, 0.51, 0.578, 0, 1.4, 1.2, 0, 0, 0, 0.537, 0.453;
    // c << 300, 460, 206, 247, 350, 400, 471, 217, 450, 246, 184, 505, 329, 356, 284, 285;
    // z << 0, 0, 0, 0, 0, 0, 100, -100, 150, -150, -200, 200, 150, 200, -150, -200;

    // x << -0.5, -0.776, -0.4, -0.51, -0.48, -1.397, 0, 0, -1.25;
    // y << 0.620, 0.43, 0.84, 0.51, 0.578, 0, 1.4, 1.2, 0;
    // c << 300, 460, 206, 471, 217, 450, 246, 184, 505;
    // z << 0, 0, 0, 130, -130, 130, -130, -200, 200;

    // x << -0.66, -0.45, -1.301, -0.475, -0.919, 0, 0, 0, 0, 0, 0, 0, -0.64, -0.54, -0.98, -0.65, 0;
    // y << 0.64, 1.271, 0.424, 0.806, 0.527, 0.43, 0.71, 0.69, 0.48, 0.51, 0.72, 0.56, 0.6, 0.84, 0.52, 0.74, 0.96;
    // c << 299, 150, 486, 112, 481, 250, 190, 340, 269, 271, 220, 232, 283, 242, 408, 275, 273;
    // z << 0, -100, 100, -200, 200, -200, -100, 50, -200, -200, -200, -100, 0, 0, 0, 0, 0;

    // x << -6.61,1.271,0.424,0.806,0.527,0.43,0.71,0.69,0.5,0.56,0.6,0.84,0.52;
    // y << 0.64,-0.45,-1.301,-0.475,-0.919,0,0,0,0,0,-0.64,-0.54,-0.98;
    // c << 299,150,486,112,481,250,190,340,280,232,283,242,408;
    // z << 0,-100,100,-200,200,-200,-100,50,-200,-100,0,0,0;

    /* 
         *  x << -6.5, -0.45, -1.301, -0.475, -0.919;
         *  y << 0.64, 1.271, 0.424, 0.806, 0.527;
         *  c << 299, 150, 486, 112, 481;
         *  z << 0, -100, 100, -200, 200;
         */

    // x << 0.625, 0.580, 0.618, 0.603, 0.512, 0.5, 0, 0.633, 0.814, 0.814, 1.327, 1.327, 0, 0;
    // y << -0.664, -0.626, -0.604, -0.79, -0.943, -1.1, -1.1, 0, 0, 0.5, -1.146, 0, -1.629, -1.478;
    // c << 360, 280, 440, 360, 631, 733, 600, 200, 230, 220, 100, 70, 690, 780;
    // z << 0, 0, 0, 0, 100, 100, 100, -100, -100, -100, -200, -200, 200, 200;

    // x << -0.664, -0.626, -0.604, -0.79, -0.943, -1.1, -1.1, 0, 0, 0.5, -1.146, 0, -1.629, -1.478;
    // y << 0.625, 0.580, 0.618, 0.603, 0.512, 0.5, 0, 0.633, 0.814, 0.814, 1.327, 1.327, 0, 0;
    // z << 0, 0, 0, 0, 100, 100, 100, -100, -100, -100, -200, -200, 200, 200;

    // x << -0.676, -0.75, -0.53, -0.636, -0.45, -1.655, -1.16, -0.46, -0.51, -0.73;
    // y << 0.687, 0.75, 0.59, 0.716, 1.395, 0.42, 0.34, 1.631, 0.51, 0.76;
    // z << -2, 150, -150, 0, -100, 100, -25, 25, -200, 200;

    // 创建点矩阵
    MatrixXf points(point_nums, 3);
    for (int i = 0; i < point_nums; ++i) {
        points(i, 0) = x(i);  // 第一列为 x
        points(i, 1) = y(i);  // 第二列为 y
        points(i, 2) = c(i);  // 第三列为 center
    }

    interpolator = Interpolator(points, z, weights);

    nh_.getParam("camera_node/front/frame_rate", frame_rate_);
    nh_.setParam("speed", 2);
    int video_feed_back_param = 1;
    nh_.getParam("video_feed_back", video_feed_back_param);
    video_feed_back = (video_feed_back_param == 1);
    ROS_INFO(TAG COLOR_MAGENTA "Video feed back %s" COLOR_RESET,
             video_feed_back ? (COLOR_RED "Enabled") : (COLOR_GREEN "Disable"));

    int lidar_visualize = 1;
    nh_.getParam("lidar_visualize", lidar_visualize);
    visualize_lidar = (lidar_visualize == 1);
    ROS_INFO(TAG COLOR_MAGENTA "Lidar visualize %s" COLOR_RESET,
             (visualize_lidar) ? (COLOR_RED "Enabled") : (COLOR_GREEN "Disabled"));

    int exit_blue_param = 1;
    nh_.getParam("exit_blue", exit_blue_param);
    exit_blue = (exit_blue_param == 1);
    ROS_INFO(TAG COLOR_MAGENTA "Exit Blue %s" COLOR_RESET,
             (exit_blue) ? (COLOR_GREEN "Enabled") : (COLOR_RED "Disabled"));

    int exit_obstacle_param = 1;
    nh_.getParam("exit_obstacle", exit_obstacle_param);
    exit_obstacle = (exit_obstacle_param == 1);
    ROS_INFO(TAG COLOR_MAGENTA "Exit Obstacle %s" COLOR_RESET,
             (exit_obstacle) ? (COLOR_GREEN "Enabled") : (COLOR_RED "Disabled"));

    float blue_negelect_time_param = 1.2;
    nh_.getParam("blue_negelect_time", blue_negelect_time_param);
    blue_negelect_time = 1000.f * blue_negelect_time_param;
    ROS_INFO(TAG COLOR_MAGENTA "blue line negelect time: %d" COLOR_RESET, blue_negelect_time);

    ROS_INFO(TAG "TraceLine constructed succeeded! ");
}

TraceLine::TraceLine(int remain_time, ros::NodeHandle &nh, TraceLineInitParams params) : Ability(remain_time, nh) {
    prev_neg_slope = Buffer<float>(5);
    prev_pos_slope = Buffer<float>(5);
    prev_angle = Buffer<int>(3);
    prev_center = Buffer<int>(6);
    blue_horizontal_times = Buffer<float>(4, 0);

    // 控制点个数
    int point_nums = params.ref_points.rows();

    VectorXf weights = params.weights;

    // 创建点矩阵
    MatrixXf points(point_nums, 3);
    for (int i = 0; i < point_nums; ++i) {
        points(i, 0) = params.ref_points.row(0)[i];  // 第一列为 x
        points(i, 1) = params.ref_points.row(1)[i];  // 第二列为 y
        points(i, 2) = params.ref_points.row(2)[i];  // 第三列为 center
    }

    interpolator = Interpolator(points, params.ref_value, weights);
    nh_.setParam("speed", 2);

    int video_feed_back_param = 1;
    nh_.getParam("path/video_feed_back", video_feed_back_param);
    if (video_feed_back_param) {
        video_feed_back = true;
    } else {
        video_feed_back = false;
    }

    if (not video_feed_back) {
        ROS_INFO(TAG COLOR_GREEN "Video feed back Disable" COLOR_RESET);
    } else {
        ROS_INFO(TAG COLOR_RED "Video feed back Enable" COLOR_RESET);
    }

    ROS_INFO(TAG "TraceLine constructed succeeded! ");
}

void TraceLine::linePreprocess() {
    if (lines_raw.empty()) {
        ROS_WARN(TAG "No line to preprocess");
    } else {
        posLines.clear();
        negLines.clear();
        for (const auto &line : lines_raw) {
            float slope = calculateSlope(line);

            if (fabs(slope) < 0.4 or fabs(slope) > 2.5) {
                continue;  // 去除横向线段
            }

            double lineLength = cv::norm(cv::Point(line[0], line[1]) - cv::Point(line[2], line[3]));
            if (slope > 0) {
                posLines.emplace_back(line, lineLength, slope,
                                      cv::Vec2i((line[0] + line[2]) / 2, (line[1] + line[3]) / 2 + upperHeight));
            } else {
                negLines.emplace_back(line, lineLength, slope,
                                      cv::Vec2i((line[0] + line[2]) / 2, (line[1] + line[3]) / 2 + upperHeight));
            }
        }
    }

    if (blue_lines_raw.empty()) {
        ROS_INFO("No blue line to preprocess");
        return;
    }

    blueLines.clear();
    for (const auto &line : blue_lines_raw) {
        float slope = calculateSlope(line);

        if (fabs(slope) > 0.5) {
            continue;  // 去除竖向线段
        }

        double lineLength = cv::norm(cv::Point(line[0], line[1]) - cv::Point(line[2], line[3]));
        blueLines.emplace_back(line, lineLength, slope,
                               cv::Vec2i((line[0] + line[2]) / 2, (line[1] + line[3]) / 2 + upperHeight));
    }
}

// 计算平均斜率的函数
pair<float, float> TraceLine::calculateAverageSlopes(int topN) {

    // 取最长的 topN 根
    sort(posLines.begin(), posLines.end(), [](const auto &a, const auto &b) { return get<1>(a) > get<1>(b); });
    sort(negLines.begin(), negLines.end(), [](const auto &a, const auto &b) { return get<1>(a) > get<1>(b); });

    // 计算正斜率的平均值
    float posAverage = 0;
    if (!posLines.empty()) {
        int count = min(topN, static_cast<int>(posLines.size()));
        for (int i = 0; i < count; i++) {
            posAverage += get<2>(posLines[i]);
        }
        posAverage /= count;  // 平均值
    }

    // 计算负斜率的平均值
    float negAverage = 0;
    if (!negLines.empty()) {
        int count = min(topN, static_cast<int>(negLines.size()));
        for (int i = 0; i < count; i++) {
            negAverage += get<2>(negLines[i]);
        }
        negAverage /= count;  // 平均值
    }

    return {negAverage, posAverage};
}

void TraceLine::getBlueLines() {
    // 检查输入图像是否为空
    if (frame.empty()) {

        cerr << "Error: Input image is empty!" << endl;
        return;
    }
    // 获取图像的高度和宽度

    // 计算下部分的高度，根据给定的比例
    cv::Rect lowerPartRect(0, frame_height - lowerHeight, frame_width, lowerHeight);
    cv::Mat lowerPart = frame(lowerPartRect);  // 提取下部分图像

    // 转换为HSV颜色空间
    cv::Mat hsv;
    cv::cvtColor(lowerPart, hsv, cv::COLOR_BGR2HSV);

    cv::Scalar lowerBlue(110, 28, 0);
    cv::Scalar upperBlue(150, 185, 255);

    cv::Mat mask;
    cv::inRange(hsv, lowerBlue, upperBlue, mask);

    int erosion_size = 1;   // 腐蚀结构元素的大小
    int dilation_size = 1;  // 膨胀结构元素的大小

    cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2 * erosion_size + 1, 2 * erosion_size + 1),
                                                cv::Point(erosion_size, erosion_size));

    cv::erode(mask, mask, element);   // 腐蚀
    cv::dilate(mask, mask, element);  // 膨胀
    // cv::imshow("mask:", mask);

    // 使用 Canny 边缘检测
    cv::Mat edges;
    cv::Canny(mask, edges, 50, 150, 3);
    // cv::imshow("edges:", edges);

    // 使用 HoughLinesP 检测线段，调大最后一个参数，识别到的直线更长
    cv::HoughLinesP(edges, blue_lines_raw, 2, CV_PI / 70, 40, 40, 50);

    // vector<vector<cv::Point>> contours;
    // cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    // // 在原始帧上用红色绘制轮廓
    // cv::drawContours(frame, contours, -1, cv::Scalar(255, 100, 0), 2);  // 红色，线宽2
}

void TraceLine::visualizeLines(const vector<cv::Vec4i> &lines, int level = 0) {
    for (size_t i = 0; i < lines.size(); i++) {
        cv::Vec4i l = lines[i];
        cv::Point start(l[0], l[1] + upperHeight);
        cv::Point end(l[2], l[3] + upperHeight);
        cv::line(frame, start, end, cv::Scalar(0, 0, 255), 2);

        if (level > 0) {
            float slope = calculateSlope(l);
            cv::Point midPoint((start.x + end.x) / 2, (start.y + end.y) / 2);
            string slopeText = "Slope: " + to_string(slope) + " Length: " + to_string(cv::norm(start - end)) + " Center"
                + to_string(midPoint.x) + "," + to_string(midPoint.y);
            if (midPoint.y > 430) {
                cout << slopeText << endl;
            }
            cv::putText(frame, slopeText, midPoint, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
        }
    }
}

/**
 * @brief 检测蓝色横线，作为阶段切换的标志，一般在返回true时，车体距离蓝线还有35cm
 * 
 */
void TraceLine::checkBlueLine() {
    if (blueLines.empty()) {
        return;
    }

    sort(blueLines.begin(), blueLines.end(), [](const auto &a, const auto &b) { return get<1>(a) > get<1>(b); });
    auto longestLine = blueLines[0];
    if (get<3>(longestLine)[1] > 425 and get<3>(longestLine)[0] > 80 and get<3>(longestLine)[0] < (frame_width - 80)) {
        if ((get<1>(longestLine) > min_blue_length and blueLines.size() > 6)
            or (get<1>(longestLine) > 130 and blueLines.size() > 8)) {
            blue_line_found = true;
            // ROS_INFO(TAG "%s", string(20, '-').c_str());
            ROS_INFO(TAG COLOR_BLUE "Road Blue Line detected!" COLOR_RESET);
            // ROS_INFO(TAG "%s", string(20, '-').c_str());
            ROS_INFO(TAG "slope : %f length: %f center_x:  %d center_y: %d blueLines size:  %d ", get<2>(longestLine),
                     get<1>(longestLine), get<3>(longestLine)[0], get<3>(longestLine)[1],
                     static_cast<int>(blueLines.size()));
            ROS_INFO(TAG "%s", string(20, '-').c_str());
        }
    }

    if (get<3>(longestLine)[1] > 380) {
        if ((get<1>(longestLine) > min_blue_length and blueLines.size() > 5)
            or (get<1>(longestLine) > 138 and blueLines.size() > 8)) {
            if (get<2>(longestLine) < 0.05) {
                blue_horizontal_times.push(1.f);
                if (blue_horizontal_times.avg() > 0.6) {
                    vertical_blue_lock = true;
                    ROS_INFO(TAG COLOR_RED "Vertical Blue Line Locked" COLOR_RESET);
                }
            }
            ROS_INFO(TAG COLOR_CYAN "slope : %f length: %f center_x:  %d center_y: %d blueLines size:  %d " COLOR_RESET,
                     get<2>(longestLine), get<1>(longestLine), get<3>(longestLine)[0], get<3>(longestLine)[1],
                     static_cast<int>(blueLines.size()));
        } else {
            blue_horizontal_times.push(0.f);
        }
    }
    // 长度大于特定最小值，并且处于屏幕下方
}

void TraceLine::getLines() {
    // 检查输入图像是否为空
    if (frame.empty()) {
        cerr << "Error: Input image is empty!" << endl;
        return;
    }
    // 获取图像的高度和宽度

    // 计算下部分的高度，根据给定的比例
    int lowerHeight = static_cast<int>(frame_height * lowerFraction);
    cv::Rect lowerPartRect(0, frame_height - lowerHeight, frame_width, lowerHeight);
    cv::Mat lowerPart = frame(lowerPartRect);  // 提取下部分图像

    // 转换为HSV颜色空间
    cv::Mat hsv;
    cv::cvtColor(lowerPart, hsv, cv::COLOR_BGR2HSV);

    // 定义白色的HSV范围
    cv::Scalar lowerWhite(0, 0, 165);     // 白色下限
    cv::Scalar upperWhite(180, 35, 255);  // 白色上限

    // 创建白色区域的掩码
    cv::Mat mask;
    cv::inRange(hsv, lowerWhite, upperWhite, mask);

    // 形态学操作 - 腐蚀和膨胀
    int erosion_size = 1;   // 腐蚀结构元素的大小
    int dilation_size = 1;  // 膨胀结构元素的大小

    cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2 * erosion_size + 1, 2 * erosion_size + 1),
                                                cv::Point(erosion_size, erosion_size));

    cv::erode(mask, mask, element);   // 腐蚀
    cv::dilate(mask, mask, element);  // 膨胀

    // 使用 Canny 边缘检测
    cv::Mat edges;
    cv::Canny(mask, edges, 50, 150, 3);

    // 使用 HoughLinesP 检测线段
    cv::HoughLinesP(edges, lines_raw, 2, CV_PI / 180, 50, 20, 10);
}

pair<float, float> TraceLine::getLineSlope() {
    // 在原始图像上绘制检测到的线段并显示斜率
    auto [neg_slope, pos_slope] = calculateAverageSlopes(3);
    return {neg_slope, pos_slope};
}

int TraceLine::getCenter() {
    sort(posLines.begin(), posLines.end(), [](const auto &a, const auto &b) { return get<1>(a) > get<1>(b); });
    sort(negLines.begin(), negLines.end(), [](const auto &a, const auto &b) { return get<1>(a) > get<1>(b); });
    // cv::line(frame, start, end, cv::Scalar(0, 0, 255), 2);
    int res = 0;
    bool valid = true;
    if (not posLines.empty()) {
        auto line = get<0>(posLines[0]);
        int x0 = line[0], y0 = line[1], x1 = line[2], y1 = line[3];
        int right_point_pos = (lowerHeight - y0) * (x1 - x0) / (y1 - y0) + x0;
        res += right_point_pos;
        // ROS_INFO(TAG "right pos : %d ", right_point_pos);
    } else {
        res += frame_width;
    }
    if (not negLines.empty()) {
        auto line = get<0>(negLines[0]);
        int x0 = line[0], y0 = line[1], x1 = line[2], y1 = line[3];
        int left_point_pos = (lowerHeight - y0) * (x1 - x0) / (y1 - y0) + x0;
        res += left_point_pos;
        // ROS_INFO(TAG "left pos : %d ", left_point_pos);
    }
    prev_center.push(res / 2);
    return prev_center.avg();
}

/**
 * @brief 基于区间划分的策略函数，未使用
 * 
 * @param left_slope 
 * @param right_slope 
 */
void TraceLine::lineSlopeStrategy_old(float left_slope, float right_slope) {
    if (left_slope == 0 and right_slope == 0) {
        ROS_WARN(TAG " No line detected");
    } else if (left_slope != 0 and right_slope != 0) {
        float sum_slope = (left_slope + right_slope) / 2;
        int angle_value = (sum_slope - 0.05) * 2500;
        angle_value = max(-200, angle_value);
        angle_value = min(angle_value, 200);
        nh_.setParam("angle", angle_value);
    } else if (left_slope == 0) {
        int angle_value = -200;
        nh_.setParam("angle", angle_value);
    } else if (right_slope == 0) {
        int angle_value = 200;
        nh_.setParam("angle", angle_value);
    }
}

void TraceLine::lineSlopeStrategy(float left_slope, float right_slope, int center) {
    if (left_slope == 0 and right_slope == 0) {
        return;
    }
    if (fabs(left_slope) > 4) {
        left_slope = 0;
    }
    if (fabs(right_slope) > 4) {
        right_slope = 0;
    }
    VectorXf slopes_center(3);
    slopes_center << left_slope, right_slope, center;
    float res = max(min(interpolator.interpolate(slopes_center), 200.f), -200.f);
    prev_angle.push(static_cast<int>(res));
    int angle_value = prev_angle.avg();

    if (vertical_blue_lock) {
        angle_value *= 0.1;
    }

    nh_.setParam("angle", angle_value);
    ROS_INFO(TAG "left slope: %lf right slope: %lf center: %d angle: %d", left_slope, right_slope, center, angle_value);
}

/**
 * @brief 不使用线段中点插值的策略函数，未使用
 * 
 * @param left_slope 
 * @param right_slope 
 */
void TraceLine::lineSlopeStrategy(float left_slope, float right_slope) {
    if (left_slope == 0 and right_slope == 0) {
        return;
    }
    if (fabs(left_slope) > 2.5) {
        left_slope = 0;
    }
    if (fabs(right_slope) > 2.5) {
        right_slope = 0;
    }
    VectorXf slopes(2);
    slopes << left_slope, right_slope;
    float res = max(min(interpolator.interpolate(slopes), 200.f), -200.f);
    prev_angle.push(static_cast<int>(res));
    int angle_value = prev_angle.avg();
    nh_.setParam("angle", angle_value);
    ROS_INFO(TAG "ANGLE: %d", angle_value);
}

// 图像处理函数
void TraceLine::imageCallback(const sensor_msgs::ImageConstPtr &msg) {
    try {
        frame = cv_bridge::toCvShare(msg, "bgr8")->image;
        // cv::resize(frame, frame, cv::Size(frame_width, frame_height));
        if (frame.empty()) {
            ROS_WARN("Empty frame received");
            return;
        }

        // detectWhite(frame);
        // visualizeLineInfo(frame.clone());

        // 流水线处理
        getLines();
        getBlueLines();
        linePreprocess();
        auto [neg_slope, pos_slope] = getLineSlope();
        auto center = getCenter();
        cv::circle(frame, cv::Point(center, frame_height - 10), 3, cv::Scalar(255, 0, 0),
                   cv::FILLED);  // 使用 cv::FILLED 填充圆

        visualizeLines(lines_raw);
        visualizeLines(blue_lines_raw, 0);
        workingTimer += 1200 / frame_rate_;
        if (workingTimer > blue_negelect_time) {
            checkBlueLine();
        }

        if (blue_line_found and exit_blue) {
            // 控制权得尽早交出。
            // 如果是循线+右转，这会导致交出过晚，右转拐弯半径不足。
            // 所以改为由各自下一步自行延时
            stop();
            // int speed, frame_rate = 10;
            // nh_.getParam("speed", speed);
            // nh_.getParam("frame_rate", frame_rate);
            // countdownTimer -= (1000 / frame_rate);
            // ROS_INFO(TAG "Blue line found");
            // ROS_INFO(TAG "time before exit: %d", countdownTimer);
            // if (countdownTimer <= 0) {
            //     stop();
            // }
        }

        if (not is_avoid_obstacle) {
            lineSlopeStrategy(neg_slope, pos_slope, center);
        }

        if (video_feed_back) {
            cv::imshow("camera_node Feed", frame);
            cv::waitKey(10);
        }

        ros::spinOnce();  // 处理 ROS 事件
    } catch (cv_bridge::Exception &e) { ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str()); }
}

const int WIDTH = 600;
const int HEIGHT = 600;
void TraceLine::visualizeLidar(vector<float> distances) {
    cv::Mat img = cv::Mat::zeros(HEIGHT, WIDTH, CV_8UC3);
    cv::Point center(WIDTH / 2, HEIGHT / 2);
    int scalar = 100;

    // 计算每个距离点的坐标并绘制
    for (size_t i = 0; i < distances.size(); ++i) {
        float angle = (2 * M_PI / distances.size()) * i;  // 计算当前点的角度
        float distance = distances[i] * scalar;

        // 将距离转换为像素坐标
        int x = static_cast<int>(center.x + distance * cos(angle));
        int y = static_cast<int>(center.y + distance * sin(angle));

        // 确保坐标在图像范围内
        if (x >= 0 && x < WIDTH && y >= 0 && y < HEIGHT) {
            img.at<cv::Vec3b>(y, x) = cv::Vec3b(0, 255, 0);  // 使用绿色绘制点
        }
    }

    // 显示图像
    cv::imshow("Lidar Visualization", img);
    cv::waitKey(1);
}

void TraceLine::laserCallback(const sensor_msgs::LaserScan::ConstPtr &msg) {
    float front_distance = findFrontDistance(msg->ranges);
    if (visualize_lidar) {
        visualizeLidar(msg->ranges);
    }
    if (front_distance > min_distance) {
        return;
    } else if (exit_obstacle) {
        ROS_INFO(TAG COLOR_RED "front distance:  %f" COLOR_RESET, front_distance);
        ROS_INFO(TAG COLOR_RED "TraceLine exit because of obstacle" COLOR_RESET);
        stop();
    }
}

float TraceLine::findFrontDistance(vector<float> ranges) {
    float distance = 50;
    for (int i = 0; i < 7; i++) {
        if (ranges[i] > 0.1f) {
            distance = min(distance, ranges[i]);
        }
    }
    int length = ranges.size();
    for (int i = length - 1; i >= length - 7; i--) {
        if (ranges[i] > 0.1f) {
            distance = min(distance, ranges[i]);
        }
    }
    if (distance >= 50) {
        return 50.f;
    } else {
        return distance;
    }
}

void TraceLine::run() {
    ROS_INFO(TAG "TraceLine started to run");
    is_running_ = true;
    sub_ = nh_.subscribe("/image_topic/front", 1, &TraceLine::imageCallback, this);
    laser_sub_ = nh_.subscribe("/scan", 1, &TraceLine::laserCallback, this);

    nh_.setParam("direction", std::string(1, 'W'));
    ros::Rate handle_rate(handle_rate_);  // 处理频率
    while (ros::ok() && is_running_) {
        ros::spinOnce();
        handle_rate.sleep();
    }
}
void TraceLine::stop() {
    is_running_ = false;
    sub_.shutdown();
    laser_sub_.shutdown();
}
TraceLine::~TraceLine() {}