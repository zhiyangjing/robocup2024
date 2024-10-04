
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

void updateParameters(int, void*) {
    // 这个函数可以为空
}

int minLineLength = 20;  // 最小线段长度
int maxLineGap = 10;     // 最大间隙
double rho = 1;          // 距离分辨率
double theta = CV_PI / 180; // 角度分辨率
int threshold = 100;     // 阈值
bool isPaused = false;   // 播放状态
int currentFrame = 0;    // 当前帧
int totalFrames = 10000;     // 视频总帧数

void detectWhite(const cv::Mat &frame) {
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

void detectLine(cv::Mat &image) {
    // 检查输入图像是否为空
    if (image.empty()) {
        std::cerr << "Error: Input image is empty!" << std::endl;
        return;
    }

    // 获取图像的高度和宽度
    int height = image.rows;
    int width = image.cols;

    // 只处理图像的下半部分
    cv::Rect lowerHalf(0, height / 2, width, height / 2);
    cv::Mat lowerPart = image(lowerHalf); // 提取下半部分图像

    // 转换为HSV颜色空间
    cv::Mat hsv;
    cv::cvtColor(lowerPart, hsv, cv::COLOR_BGR2HSV);

    // 定义白色的HSV范围
    cv::Scalar lowerWhite(0, 0, 185);  // 白色下限
    cv::Scalar upperWhite(180, 30, 255); // 白色上限

    // 创建白色区域的掩码
    cv::Mat mask;
    cv::inRange(hsv, lowerWhite, upperWhite, mask);

    // 形态学操作 - 腐蚀和膨胀
    int erosion_size = 1; // 腐蚀结构元素的大小
    int dilation_size = 1; // 膨胀结构元素的大小

    cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, 
                        cv::Size(2 * erosion_size + 1, 2 * erosion_size + 1),
                        cv::Point(erosion_size, erosion_size));

    cv::erode(mask, mask, element); // 腐蚀
    cv::dilate(mask, mask, element); // 膨胀

    // 使用 Canny 边缘检测
    cv::Mat edges;
    cv::Canny(mask, edges, 50, 150, 3);

    // 存储检测到的线段
    std::vector<cv::Vec4i> lines;
    // 使用 HoughLinesP 检测线段
    cv::HoughLinesP(edges, lines, rho, theta, threshold, minLineLength, maxLineGap);

    // 在下半部分的原始图像上绘制绿色轮廓
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    for (size_t i = 0; i < contours.size(); i++) {
        cv::Scalar greenColor(0, 255, 0); // 绿色
        // 添加 Y 轴偏移
        for (size_t j = 0; j < contours[i].size(); j++) {
            contours[i][j].y += height / 2; // Y 轴偏移
        }
        cv::drawContours(image, contours, static_cast<int>(i), greenColor, 2, cv::LINE_8);
    }

    // 在原始图像上绘制检测到的线段
    for (size_t i = 0; i < lines.size(); i++) {
        cv::Vec4i l = lines[i];
        // 由于我们只处理下半部分，所以线段的起始点和终止点需要加上下半部分的y坐标偏移
        cv::line(image, cv::Point(l[0], l[1] + height / 2), cv::Point(l[2], l[3] + height / 2), cv::Scalar(0, 0, 255), 2); // 绘制线段
    }
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

public:
    TraceLine(int remain_time, ros::NodeHandle &nh) : Ability(remain_time, nh) {
        sub_ = nh_.subscribe("image_topic", 1, &TraceLine::imageCallback, this);
    }

    // 计算并显示帧率的函数
    void imageCallback(const sensor_msgs::ImageConstPtr &msg) {
        try {
            cv::Mat frame = cv_bridge::toCvShare(msg, "bgr8")->image;

            cv::resize(frame, frame, cv::Size(frame_width, frame_height));
            if (frame.empty()) {
                ROS_WARN("Empty frame received");
                return;
            }

            // detectWhite(frame);
            detectLine(frame);

            ROS_INFO("%s image processed", TAG);

            cv::imshow("camera_node Feed", frame);
            cv::waitKey(30);

            ros::spinOnce();// 处理 ROS 事件
        } catch (cv_bridge::Exception &e) {
            ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
        }
    }

    void run() {

    }
};


int main(int argc, char **argv) {
    ros::init(argc, argv, "trace_line");
    ros::NodeHandle nh;
    
    cv::namedWindow("camera_node Feed", cv::WINDOW_AUTOSIZE);


    cv::createTrackbar("Min Line Length", "camera_node Feed", &minLineLength, 200, updateParameters);
    cv::createTrackbar("Max Line Gap", "camera_node Feed", &maxLineGap, 50, updateParameters);
    cv::createTrackbar("Rho", "camera_node Feed", reinterpret_cast<int*>(&rho), 10, updateParameters);
    cv::createTrackbar("Theta", "camera_node Feed", reinterpret_cast<int*>(&theta), 180, updateParameters);
    cv::createTrackbar("Threshold", "camera_node Feed", &threshold, 200, updateParameters);
    cv::createTrackbar("Frame", "camera_node Feed", &currentFrame, totalFrames - 1, updateParameters); // 视频进度


    TraceLine lightProcessor(10, nh);

    ros::spin();
    cv::destroyAllWindows();
    return 0;
}