#include <chrono>
#include <common_utils/common.h>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

using namespace std;
enum States { BIG_LEFT_TURN = 0, LIGHT_DETECT, TRACE_LINE, ROAD_LEFT_TURN, SMALL_LEFT_TURN, TERMINAL };

class Ability {
public:
    virtual void run() = 0;
    Ability(int remain_time, ros::NodeHandle nh) : remain_time_(remain_time), nh_(nh) {};

protected:
    ros::NodeHandle nh_;
    int remain_time_;
};

struct TraceLineInitParams {
    MatrixXf ref_points;
    VectorXf ref_value;
    VectorXf weights;
};

class TraceLine : public Ability {
private:
    ros::Subscriber sub_;
    bool is_running_ = false;
    bool blue_line_found = false;
    int frame_height = 480;
    int frame_width = 720;
    int countdownTimer = 1000;  //  单位毫秒，在识别到蓝色线条之后剩余的运行时间
    int min_blue_length = 300;
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
    vector<cv::Vec4i> lines_raw;       // 存储检测到的白色车道线段
    vector<cv::Vec4i> blue_lines_raw;  // 存储检测到的蓝色线段

    vector<tuple<cv::Vec4i, float, float, cv::Vec2i>> posLines;   // 线段，长度，斜率 , 中点
    vector<tuple<cv::Vec4i, float, float, cv::Vec2i>> negLines;   // 线段，长度，斜率 , 中点
    vector<tuple<cv::Vec4i, float, float, cv::Vec2i>> blueLines;  // 线段，长度，斜率 , 中点

public:
    TraceLine(int remain_time, ros::NodeHandle &nh);
    TraceLine(int remain_time, ros::NodeHandle &nh, TraceLineInitParams params);
    void linePreprocess();
    void getLines();
    void getBlueLines();
    int getCenter();
    pair<float, float> calculateAverageSlopes(int topN);
    pair<float, float> getLineSlope();
    void lineSlopeStrategy_old(float left_slope, float right_slope);
    void lineSlopeStrategy(float left_slope, float right_slope, int center);
    void lineSlopeStrategy(float left_slope, float right_slope);
    void imageCallback(const sensor_msgs::ImageConstPtr &msg);
    void checkBlueLine();
    void run();
    void stop();
    ~TraceLine();
};

class LightDetector : public Ability {
private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    bool is_running_ = false;

    int frameCount = 0;
    std::chrono::steady_clock::time_point lastTime = std::chrono::steady_clock::now();
    int frame_height = 480;
    int frame_width = 720;
    int line_pos = frame_width * 0.422;
    double currentFps = 0;
    Buffer<int> light_states;

    // HSV阈值设置
    cv::Scalar lower_red1 = cv::Scalar(0, 100, 100);
    cv::Scalar high_red1 = cv::Scalar(10, 255, 255);
    cv::Scalar lower_red2 = cv::Scalar(160, 100, 100);
    cv::Scalar high_red2 = cv::Scalar(180, 255, 255);
    cv::Scalar lower_green = cv::Scalar(40, 100, 100);
    cv::Scalar high_green = cv::Scalar(80, 255, 255);
    cv::Mat hsv_frame, green_mask;
    cv::Mat mask1, mask2, red_mask;

public:
    LightDetector(int remain_time, ros::NodeHandle &nh);
    void run();
    void stop();
    void imageCallback(const sensor_msgs::ImageConstPtr &msg);
    void detectTrafficLights(cv::Mat &frame, ros::NodeHandle &nh);
};