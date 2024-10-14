#include <chrono>
#include <common_utils/common.h>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <path/path.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#define TAG "[Trace Line]"

using namespace std;

void detectWhite(const cv::Mat &frame) {
    ROS_INFO(COLOR_RED TAG COLOR_RESET " detect white ");
    // 将输入的 BGR 图像转为 HSV（色相、饱和度、亮度）颜色空间
    cv::Mat hsvFrame;
    cv::cvtColor(frame, hsvFrame, cv::COLOR_BGR2HSV);

    // 定义白色的阈值范围
    // 白色的 HSV 值接近 (0, 0, 255)，因此饱和度 (S) 要低，亮度 (V) 要高
    cv::Scalar lowerWhite(0, 0, 185);   // 最低值 (0, 0, 200)
    cv::Scalar upperWhite(180, 30, 255);// 最高值 (180, 55, 255)

    // 创建白色部分的遮罩
    cv::Mat mask;
    cv::inRange(hsvFrame, lowerWhite, upperWhite, mask);

    // 查找遮罩中的轮廓
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    // 在原始帧上用红色绘制轮廓
    cv::drawContours(frame, contours, -1, cv::Scalar(0, 0, 255), 2);// 红色，线宽2
}

void preprocess(cv::Mat frame) {}

float calculateSlope(const cv::Vec4i &line) {
    float dx = line[2] - line[0];
    float dy = line[3] - line[1];
    if (dx == 0) {
        return std::numeric_limits<float>::infinity();// 处理垂直线
    }
    return dy / dx;// 计算斜率
}

/**
 * @brief 这是一个辅助函数，用于可视化图像
 * @details 标注出识别到的白色区域，同时用红色标注出识别到的线条以及斜率
 * 
 * 
 * @param image 
 */
void detectLine(cv::Mat image) {
    // 检查输入图像是否为空
    if (image.empty()) {
        std::cerr << "Error: Input image is empty!" << std::endl;
        return;
    }
    float lowerFraction = 0.4;
    // 获取图像的高度和宽度
    int height = image.rows;
    int width = image.cols;

    // 计算下部分的高度，根据给定的比例
    int lowerHeight = static_cast<int>(height * lowerFraction);
    cv::Rect lowerPartRect(0, height - lowerHeight, width, lowerHeight);
    cv::Mat lowerPart = image(lowerPartRect);// 提取下部分图像

    // 转换为HSV颜色空间
    cv::Mat hsv;
    cv::cvtColor(lowerPart, hsv, cv::COLOR_BGR2HSV);

    // 定义白色的HSV范围
    cv::Scalar lowerWhite(0, 0, 185);   // 白色下限
    cv::Scalar upperWhite(180, 30, 255);// 白色上限

    // 创建白色区域的掩码
    cv::Mat mask;
    cv::inRange(hsv, lowerWhite, upperWhite, mask);

    // 形态学操作 - 腐蚀和膨胀
    int erosion_size = 1; // 腐蚀结构元素的大小
    int dilation_size = 1;// 膨胀结构元素的大小

    cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2 * erosion_size + 1, 2 * erosion_size + 1),
                                                cv::Point(erosion_size, erosion_size));

    cv::erode(mask, mask, element); // 腐蚀
    cv::dilate(mask, mask, element);// 膨胀

    // 使用 Canny 边缘检测
    cv::Mat edges;
    cv::Canny(mask, edges, 50, 150, 3);
    cv::imshow("canny", edges);

    // 存储检测到的线段
    std::vector<cv::Vec4i> lines;
    // 使用 HoughLinesP 检测线段
    cv::HoughLinesP(edges, lines, 2, CV_PI / 180, 50, 30, 10);

    // 在下半部分的原始图像上绘制绿色轮廓
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    for (size_t i = 0; i < contours.size(); i++) {
        cv::Scalar greenColor(0, 255, 0);// 绿色
        for (size_t j = 0; j < contours[i].size(); j++) {
            contours[i][j].y += height - lowerHeight;// Y 轴偏移
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
        std::string slopeText = "Slope: " + std::to_string(slope);
        cv::Point midPoint((start.x + end.x) / 2, (start.y + end.y) / 2);
        cv::putText(image, slopeText, midPoint, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
    }

    cv::imshow("detect line result", image);
}

// 计算平均斜率的函数
std::pair<float, float> calculateAverageSlopes(const std::vector<cv::Vec4i> &lines, int topN) {
    std::vector<float> posSlopes;
    std::vector<float> negSlopes;

    // 根据斜率将线段分组
    for (const auto &line : lines) {
        float slope = calculateSlope(line);
        double lineLength = cv::norm(cv::Point(line[0], line[1]) - cv::Point(line[2], line[3]));

        if (fabs(slope) < 0.35) {
            continue;// 去除横向线段
        }

        if (slope > 0) {
            posSlopes.push_back(slope);
        } else {
            negSlopes.push_back(slope);
        }
    }

    // 对斜率按绝对值排序，并取最长的 topN 根
    std::sort(posSlopes.begin(), posSlopes.end(), std::greater<float>());
    std::sort(negSlopes.begin(), negSlopes.end());

    // 计算正斜率的平均值
    float posAverage = 0;
    if (!posSlopes.empty()) {
        int count = std::min(topN, static_cast<int>(posSlopes.size()));
        for (int i = 0; i < count; i++) {
            posAverage += posSlopes[i];
        }
        posAverage /= count;// 平均值
    }

    // 计算负斜率的平均值
    float negAverage = 0;
    if (!negSlopes.empty()) {
        int count = std::min(topN, static_cast<int>(negSlopes.size()));
        for (int i = 0; i < count; i++) {
            negAverage += negSlopes[i];
        }
        negAverage /= count;// 平均值
    }

    return {negAverage, posAverage};
}

pair<float, float> getLineSlope(cv::Mat &image) {
    // 检查输入图像是否为空
    if (image.empty()) {
        std::cerr << "Error: Input image is empty!" << std::endl;
        return {0.f, 0.f};
    }
    float lowerFraction = 0.4;
    // 获取图像的高度和宽度
    int height = image.rows;
    int width = image.cols;

    // 计算下部分的高度，根据给定的比例
    int lowerHeight = static_cast<int>(height * lowerFraction);
    cv::Rect lowerPartRect(0, height - lowerHeight, width, lowerHeight);
    cv::Mat lowerPart = image(lowerPartRect);// 提取下部分图像

    // 转换为HSV颜色空间
    cv::Mat hsv;
    cv::cvtColor(lowerPart, hsv, cv::COLOR_BGR2HSV);

    // 定义白色的HSV范围
    cv::Scalar lowerWhite(0, 0, 185);   // 白色下限
    cv::Scalar upperWhite(180, 30, 255);// 白色上限

    // 创建白色区域的掩码
    cv::Mat mask;
    cv::inRange(hsv, lowerWhite, upperWhite, mask);

    // 形态学操作 - 腐蚀和膨胀
    int erosion_size = 1; // 腐蚀结构元素的大小
    int dilation_size = 1;// 膨胀结构元素的大小

    cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2 * erosion_size + 1, 2 * erosion_size + 1),
                                                cv::Point(erosion_size, erosion_size));

    cv::erode(mask, mask, element); // 腐蚀
    cv::dilate(mask, mask, element);// 膨胀

    // 使用 Canny 边缘检测
    cv::Mat edges;
    cv::Canny(mask, edges, 50, 150, 3);

    // 存储检测到的线段
    std::vector<cv::Vec4i> lines;
    // 使用 HoughLinesP 检测线段
    cv::HoughLinesP(edges, lines, 2, CV_PI / 180, 50, 20, 10);

    for (size_t i = 0; i < lines.size(); i++) {
        cv::Vec4i l = lines[i];
        cv::Point start(l[0], l[1] + (height - lowerHeight));
        cv::Point end(l[2], l[3] + (height - lowerHeight));
        cv::line(image, start, end, cv::Scalar(0, 0, 255), 2);
    }
    // 在原始图像上绘制检测到的线段并显示斜率
    auto [neg_slope, pos_slope] = calculateAverageSlopes(lines, 3);
    return {neg_slope, pos_slope};
}

class TraceLine : public Ability {
private:
    ros::Subscriber sub_;
    int frameCount = 0;
    std::chrono::steady_clock::time_point lastTime = std::chrono::steady_clock::now();
    int frame_height = 480;
    int frame_width = 720;
    int line_pos = frame_width * 0.422;
    double currentFps = 0;
    Buffer<float> prev_neg_slope;
    Buffer<float> prev_pos_slope;
    Buffer<int> prev_angle;
    Interpolator interpolator;

public:
    TraceLine(int remain_time, ros::NodeHandle &nh) : Ability(remain_time, nh) {
        prev_neg_slope = Buffer<float>(5);
        prev_pos_slope = Buffer<float>(5);
        prev_angle = Buffer<int>(10);

        VectorXf x(10);
        VectorXf y(10);
        VectorXf z(10);

        x << -0.676, -0.75, -0.53, -0.636, -0.45, -1.655, -1.16, -0.46, -0.51, -0.73;
        y << 0.687, 0.75, 0.59, 0.716, 1.395, 0.42, 0.34, 1.631, 0.51, 0.76;
        z << -2, 150, -150, 0, -100, 100, -25, 25, -200, 200;

        // 创建点矩阵
        MatrixXf points(10, 2);
        for (int i = 0; i < 10; ++i) {
            points(i, 0) = x(i);// 第一列为 x
            points(i, 1) = y(i);// 第二列为 y
        }

        interpolator = Interpolator(points, z);
    }

    void lineSlopeStrategy_old(float left_slope, float right_slope) {
        if (left_slope == 0 and right_slope == 0) {
            ROS_WARN(TAG " No line detected");
        } else if (left_slope != 0 and right_slope != 0) {
            float sum_slope = (left_slope + right_slope) / 2;
            int angle_value = (sum_slope - 0.05) * 2500;
            // ROS_INFO(TAG "mid part %d", angle_value);
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

    void lineSlopeStrategy(float left_slope, float right_slope) {
        if (left_slope == 0 and right_slope == 0) {
            return;
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
    void imageCallback(const sensor_msgs::ImageConstPtr &msg) {
        try {
            cv::Mat frame = cv_bridge::toCvShare(msg, "bgr8")->image;

            cv::resize(frame, frame, cv::Size(frame_width, frame_height));
            if (frame.empty()) {
                ROS_WARN("Empty frame received");
                return;
            }

            // detectWhite(frame);
            // detectLine(frame.clone());

            auto [neg_slope, pos_slope] = getLineSlope(frame);
            /*
             * // 差错控制代码，将这一部分移入了lineSlopeStragety
             * if (fabs(neg_slope) < 0.001) {
             *     neg_slope = prev_neg_slope[0];
             * }
             * prev_neg_slope.push(neg_slope);
             * 
             * if (fabs(pos_slope) < 0.001) {
             *     pos_slope = prev_pos_slope[0];
             * }
             * prev_pos_slope.push(pos_slope);
             */

            lineSlopeStrategy(neg_slope, pos_slope);
            // ROS_INFO(TAG "left slope: %lf right slope %lf ", neg_slope, pos_slope);

            cv::imshow("camera_node Feed", frame);
            cv::waitKey(30);

            ros::spinOnce();// 处理 ROS 事件
        } catch (cv_bridge::Exception &e) {
            ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
        }
    }

    void run() { sub_ = nh_.subscribe("image_topic", 1, &TraceLine::imageCallback, this); }
    void stop() {
        // 使用空的订阅来取消之前订阅
        sub_ = ros::Subscriber();
    }
    ~TraceLine() {}
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

    TraceLine lightProcessor(10, nh);
    lightProcessor.run();

    ros::spin();
    cv::destroyAllWindows();
    return 0;
}