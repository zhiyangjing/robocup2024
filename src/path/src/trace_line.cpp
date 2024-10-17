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
    cv::Scalar lowerWhite(0, 0, 160);   // 最低值 (0, 0, 200)
    cv::Scalar upperWhite(180, 30, 255);// 最高值 (180, 55, 255)

    // 创建白色部分的遮罩
    cv::Mat mask;
    cv::inRange(hsvFrame, lowerWhite, upperWhite, mask);

    // 查找遮罩中的轮廓
    vector<vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    // 在原始帧上用红色绘制轮廓
    cv::drawContours(frame, contours, -1, cv::Scalar(0, 0, 255), 2);// 红色，线宽2
}

void preprocess(cv::Mat frame) {}

float calculateSlope(const cv::Vec4i &line) {
    float dx = line[2] - line[0];
    float dy = line[3] - line[1];
    if (dx == 0) {
        return numeric_limits<float>::infinity();// 处理垂直线
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
    cv::Mat lowerPart = image(lowerPartRect);// 提取下部分图像

    // 转换为HSV颜色空间
    cv::Mat hsv;
    cv::cvtColor(lowerPart, hsv, cv::COLOR_BGR2HSV);

    // 定义白色的HSV范围
    cv::Scalar lowerWhite(0, 0, 160);   // 白色下限
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
    vector<cv::Vec4i> lines;
    // 使用 HoughLinesP 检测线段
    cv::HoughLinesP(edges, lines, 2, CV_PI / 180, 50, 30, 10);

    // 在下半部分的原始图像上绘制绿色轮廓
    vector<vector<cv::Point>> contours;
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
        string slopeText = "Slope: " + to_string(slope);
        cv::Point midPoint((start.x + end.x) / 2, (start.y + end.y) / 2);
        cv::putText(image, slopeText, midPoint, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
    }

    cv::imshow("detect line result", image);
}

class TraceLine : public Ability {
private:
    ros::Subscriber sub_;
    int frame_height = 480;
    int frame_width = 720;
    float lowerFraction = 0.4;
    int lowerHeight = static_cast<int>(frame_height * lowerFraction);
    int line_pos = frame_width * 0.422;
    double currentFps = 0;
    Buffer<float> prev_neg_slope;
    Buffer<float> prev_pos_slope;
    Buffer<int> prev_angle;
    Buffer<int> prev_center;
    Interpolator interpolator;
    cv::Mat frame;
    vector<cv::Vec4i> lines;                                   // 存储检测到的线段
    vector<tuple<cv::Vec4i, float, float, cv::Vec2i>> posLines;// 线段，长度，斜率 , 中点
    vector<tuple<cv::Vec4i, float, float, cv::Vec2i>> negLines;

public:
    TraceLine(int remain_time, ros::NodeHandle &nh) : Ability(remain_time, nh) {
        prev_neg_slope = Buffer<float>(5);
        prev_pos_slope = Buffer<float>(5);
        prev_angle = Buffer<int>(3);
        prev_center = Buffer<int>(10);

        // 控制点个数
        int point_nums = 17;

        VectorXf weights(3);
        VectorXf x(point_nums);
        VectorXf y(point_nums);
        VectorXf c(point_nums);
        VectorXf z(point_nums);
        weights << 1, 1, 100;// 斜率的权重是1，中心点的权重是1.5
        x << -0.66, -0.45, -1.301, -0.475, -0.919, 0, 0, 0, 0, 0, 0, 0, -0.64, -0.54, -0.98, -0.65, 0;
        y << 0.64, 1.271, 0.424, 0.806, 0.527, 0.43, 0.71, 0.69, 0.48, 0.51, 0.72, 0.56, 0.6, 0.84, 0.52, 0.74, 0.96;
        c << 299, 150, 486, 112, 481, 250, 190, 340, 269, 271, 220, 232, 283, 242, 408, 275, 273;
        z << 0, -100, 100, -200, 200, -200, -100, 50, -200, -200, -200, -100, 0, 0, 0, 0, 0;

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
            points(i, 0) = x(i);// 第一列为 x
            points(i, 1) = y(i);// 第二列为 y
            points(i, 2) = c(i);// 第三列为 center
        }

        interpolator = Interpolator(points, z, weights);
        ROS_INFO(TAG "TraceLine constructed succeeded! ");
    }

    void linePreprocess() {
        if (lines.empty()) {
            ROS_WARN(TAG "No line to preprocess");
            return;
        }

        posLines.clear();
        negLines.clear();
        for (const auto &line : lines) {
            float slope = calculateSlope(line);

            if (fabs(slope) < 0.4 or fabs(slope) > 2.5) {
                continue;// 去除横向线段
            }

            double lineLength = cv::norm(cv::Point(line[0], line[1]) - cv::Point(line[2], line[3]));
            if (slope > 0) {
                posLines.emplace_back(line, lineLength, slope,
                                      cv::Vec2i((line[0] + line[2]) / 2, (line[1] + line[3]) / 2));
            } else {
                negLines.emplace_back(line, lineLength, slope,
                                      cv::Vec2i((line[0] + line[2]) / 2, (line[1] + line[3]) / 2));
            }
        }
    }

    // 计算平均斜率的函数
    pair<float, float> calculateAverageSlopes(int topN) {

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
            posAverage /= count;// 平均值
        }

        // 计算负斜率的平均值
        float negAverage = 0;
        if (!negLines.empty()) {
            int count = min(topN, static_cast<int>(negLines.size()));
            for (int i = 0; i < count; i++) {
                negAverage += get<2>(negLines[i]);
            }
            negAverage /= count;// 平均值
        }

        return {negAverage, posAverage};
    }

    void getLines() {
        // 检查输入图像是否为空
        if (frame.empty()) {
            cerr << "Error: Input image is empty!" << endl;
            return;
        }
        // 获取图像的高度和宽度

        // 计算下部分的高度，根据给定的比例
        int lowerHeight = static_cast<int>(frame_height * lowerFraction);
        cv::Rect lowerPartRect(0, frame_height - lowerHeight, frame_width, lowerHeight);
        cv::Mat lowerPart = frame(lowerPartRect);// 提取下部分图像

        // 转换为HSV颜色空间
        cv::Mat hsv;
        cv::cvtColor(lowerPart, hsv, cv::COLOR_BGR2HSV);

        // 定义白色的HSV范围
        cv::Scalar lowerWhite(0, 0, 170);   // 白色下限
        cv::Scalar upperWhite(180, 30, 255);// 白色上限

        // 创建白色区域的掩码
        cv::Mat mask;
        cv::inRange(hsv, lowerWhite, upperWhite, mask);

        // 形态学操作 - 腐蚀和膨胀
        int erosion_size = 1; // 腐蚀结构元素的大小
        int dilation_size = 1;// 膨胀结构元素的大小

        cv::Mat element =
            cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2 * erosion_size + 1, 2 * erosion_size + 1),
                                      cv::Point(erosion_size, erosion_size));

        cv::erode(mask, mask, element); // 腐蚀
        cv::dilate(mask, mask, element);// 膨胀

        // 使用 Canny 边缘检测
        cv::Mat edges;
        cv::Canny(mask, edges, 50, 150, 3);

        // 使用 HoughLinesP 检测线段
        cv::HoughLinesP(edges, lines, 2, CV_PI / 180, 50, 20, 10);
    }

    pair<float, float> getLineSlope() {
        for (size_t i = 0; i < lines.size(); i++) {
            cv::Vec4i l = lines[i];
            cv::Point start(l[0], l[1] + (frame_height - lowerHeight));
            cv::Point end(l[2], l[3] + (frame_height - lowerHeight));
            cv::line(frame, start, end, cv::Scalar(0, 0, 255), 2);
        }
        // 在原始图像上绘制检测到的线段并显示斜率
        auto [neg_slope, pos_slope] = calculateAverageSlopes(3);
        return {neg_slope, pos_slope};
    }

    int getCenter() {
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

    void lineSlopeStrategy_old(float left_slope, float right_slope) {
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

    void lineSlopeStrategy(float left_slope, float right_slope, int center) {
        if (left_slope == 0 and right_slope == 0) {
            return;
        }
        if (fabs(left_slope) > 2.5) {
            left_slope = 0;
        }
        if (fabs(right_slope) > 2.5) {
            right_slope = 0;
        }
        VectorXf slopes_center(3);
        slopes_center << left_slope, right_slope, center;
        float res = max(min(interpolator.interpolate(slopes_center), 200.f), -200.f);
        prev_angle.push(static_cast<int>(res));
        int angle_value = prev_angle.avg();
        nh_.setParam("angle", angle_value);
        ROS_INFO(TAG "ANGLE: %d", angle_value);
    }

    void lineSlopeStrategy(float left_slope, float right_slope) {
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
    void imageCallback(const sensor_msgs::ImageConstPtr &msg) {
        try {
            frame = cv_bridge::toCvShare(msg, "bgr8")->image;
            cv::resize(frame, frame, cv::Size(frame_width, frame_height));
            if (frame.empty()) {
                ROS_WARN("Empty frame received");
                return;
            }

            // detectWhite(frame);
            // visualizeLineInfo(frame.clone());

            // 流水线处理
            getLines();
            linePreprocess();
            auto [neg_slope, pos_slope] = getLineSlope();
            auto center = getCenter();
            cv::circle(frame, cv::Point(center, frame_height - 10), 3, cv::Scalar(255, 0, 0),
                       cv::FILLED);// 使用 cv::FILLED 填充圆

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

            lineSlopeStrategy(neg_slope, pos_slope, center);
            ROS_INFO(TAG "left slope: %lf right slope: %lf center: %d", neg_slope, pos_slope, center);

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