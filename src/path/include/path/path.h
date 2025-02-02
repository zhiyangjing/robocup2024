#include <chrono>
#include <common_utils/common.h>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/LaserScan.h>
#include <string>

using namespace std;
enum States {
    BIG_LEFT_TURN = 0,
    LIGHT_DETECT,
    TRACE_LINE,
    AVOID,
    UTURN,
    ROAD_LEFT_TURN,
    ROAD_RIGHT_TURN,
    REVERSE_PARK,
    FORWARD_PARK,
    STRAIGHT,
    SMALL_LEFT_TURN,
    TERMINAL,
    SIDE_PARK
};


/**
 * @brief 一个抽象类，由此产生所有的其他能力(巡线、倒车等等)
 * 
 */
class Ability {
public:
    virtual void run() = 0;
    Ability(int remain_time, ros::NodeHandle nh) : remain_time_(remain_time), nh_(nh) {};

protected:
    ros::NodeHandle nh_;
    int remain_time_;
};


/**
 * \ingroup TraceLine
 * @brief 
 * 
 */
struct TraceLineInitParams {
    /*! \~chinese 参考点的个数 */
    MatrixXf ref_points; 
    VectorXf ref_value; //< 参考点的具体值
    VectorXf weights; // 参考点的权重
};



/**
 * \ingroup Park
 * @brief 用于给Park类传参的类，根据不同的param实现fowardPark，backwardPark
 */
struct ParkInitParams {
    MatrixXf ref_points;      // 参考点坐标
    VectorXf ref_value;       // 参考点对应权重
    VectorXf weights;         // 权重
    float first_stage_param;  // 一阶段计算转动角度的参数，正数或者负数，合适的范围约为：[-4,4]
    string camera;
    float lowerFraction;

    ParkInitParams() {};

    /**
     * @brief Construct a new Park Init Params object
     * 
     * @param num_points 参考点个数
     * @param num_dimensions 参考点维度
     */
    ParkInitParams(int num_points, int num_dimensions) {
        ref_points.resize(num_dimensions, num_points);  // 3 行 num_points 列
        ref_value.resize(num_points);                   // num_points 大小
        weights.resize(num_dimensions);                 // num_weights 大小
    }

    ParkInitParams(const ParkInitParams &other)
        : ref_points(other.ref_points), ref_value(other.ref_value), weights(other.weights),
          first_stage_param(other.first_stage_param), camera(other.camera), lowerFraction(other.lowerFraction) {}

    // 赋值运算符重载
    ParkInitParams &operator=(const ParkInitParams &other) {
        if (this != &other) {  // 自我赋值检查
            ref_points = other.ref_points;
            ref_value = other.ref_value;
            weights = other.weights;
            first_stage_param = other.first_stage_param;
            camera = other.camera;
            lowerFraction = other.lowerFraction;
        }
        return *this;  // 返回当前对象的引用
    }
};

class SidePark : public Ability {
private:
    ros::Subscriber sub_;        // 图像的订阅者
    ros::Subscriber laser_sub_;  // 雷达信息的订阅
    // ---------------------------------
    bool turning_stage = false;
    bool straight_stage = false;
    bool is_straight = false;  // 是否对正
    // ---------------------------------
    bool is_running_ = false;
    bool blue_line_found = false;  // 只有在足够靠近智能车才会被设置为true
    bool exit_blue = true;         // 遇到蓝线时是否退出，测试时使用
    bool exit_obstacle = true;     // 遇到障碍物是否退出，测试使用
    bool is_avoid_obstacle = false;
    bool video_feed_back = true;
    bool visualize_lidar = true;
    bool vertical_blue_lock = false;  // 蓝线垂直锁定，用于优化路口方向，检测到蓝线斜率接近0，则不再调整斜率
    bool enable_blue_lock = true;
    bool dir_adjust_finish = false;
    bool blue_line_visible = false;
    int frame_height = 480;
    int frame_width = 640;
    int countdownTimer = 200;  //  单位毫秒，在识别到蓝色线条之后剩余的运行时间
    int workingTimer = 0;      // 记录已经运行的时间，开始的一段时间内不识别蓝线，单位毫秒
    int blue_negelect_time = 1200;  // 不识别蓝线的时间
    int min_blue_length = 220;
    float min_distance = 0.9;
    float lowerFraction = 0.4;
    int lowerHeight = static_cast<int>(frame_height * lowerFraction);
    int upperHeight = frame_height - lowerHeight;
    int line_pos = frame_width * 0.422;
    int handle_rate_ = 20;  // 应该大于frame_rate
    int frame_rate_ = 10;   // 通过param获取的frame_rate
    double currentFps = 0;
    Buffer<float> prev_neg_slope;
    Buffer<float> prev_pos_slope;
    Buffer<int> prev_angle;
    Buffer<int> prev_center;
    Buffer<float> blue_horizontal_times;
    Buffer<float> blue_line_slopes;  // 不会检查高度，斜率满足小于0.5，只是最长
    // ----------------------
    Buffer<int> prev_angle_long_term;
    // ----------------------
    Interpolator interpolator;
    cv::Mat frame;
    vector<cv::Vec4i> lines_raw;                                 // 存储检测到的白色车道线段
    vector<cv::Vec4i> lidar_lines_raw;                           // 存储检测到的蓝色线段
    vector<tuple<cv::Vec4i, float, float, cv::Vec2i>> posLines;  // 线段，长度，斜率 , 中点
    vector<tuple<cv::Vec4i, float, float, cv::Vec2i>> negLines;  // 线段，长度，斜率 , 中点
    vector<tuple<cv::Vec4i, float, float, cv::Vec2i>> lidarLines;

public:
    SidePark(int remain_time, ros::NodeHandle &nh);
    void linePreprocess();
    void getLines();
    int getCenter();
    float calculateSlope(const cv::Vec4i &line);
    pair<float, float> calculateAverageSlopes(int topN);
    pair<float, float> getLineSlope();
    void lineSlopeStrategy_old(float left_slope, float right_slope);
    void find_target(float left_slope, float right_slope, int center);
    void lineSlopeStrategy(float left_slope, float right_slope, int center);
    void lineSlopeStrategy(float left_slope, float right_slope);
    void imageCallback(const sensor_msgs::ImageConstPtr &msg);
    void laserCallback(const sensor_msgs::LaserScan::ConstPtr &msg);
    float findFrontDistance(vector<float> ranges);
    void visualizeLines(const vector<cv::Vec4i> &lines, int level);
    void get_lidar_line(vector<float> distances);
    void lidarLinePreprocess();
    void run();
    void stop();
    ~SidePark();
};


/**
 * \ingroup TraceLine
 * @brief 
 * 
 */
class TraceLine : public Ability {
private:
    ros::Subscriber sub_;        // 图像的订阅者
    ros::Subscriber laser_sub_;  // 雷达信息的订阅
    bool is_running_ = false;
    bool blue_line_found = false;  // 只有在足够靠近智能车才会被设置为true
    bool exit_blue = true;         // 遇到蓝线时是否退出，测试时使用
    bool exit_obstacle = true;     // 遇到障碍物是否退出，测试使用
    bool is_avoid_obstacle = false;
    bool video_feed_back = true;
    bool visualize_lidar = true;
    bool vertical_blue_lock = false;  // 蓝线垂直锁定，用于优化路口方向，检测到蓝线斜率接近0，则不再调整斜率
    bool enable_blue_lock = true;
    bool dir_adjust_finish = false;
    bool blue_line_visible = false;
    int frame_height = 480;
    int frame_width = 640;
    int countdownTimer = 200;  //  单位毫秒，在识别到蓝色线条之后剩余的运行时间
    int workingTimer = 0;      // 记录已经运行的时间，开始的一段时间内不识别蓝线，单位毫秒
    int blue_negelect_time = 1200;  // 不识别蓝线的时间
    int min_blue_length = 220;
    float min_distance = 0.9;
    float lowerFraction = 0.4;
    int lowerHeight = static_cast<int>(frame_height * lowerFraction);
    int upperHeight = frame_height - lowerHeight;
    int line_pos = frame_width * 0.422;
    int handle_rate_ = 20;  // 应该大于frame_rate
    int frame_rate_ = 10;   // 通过param获取的frame_rate
    double currentFps = 0;
    Buffer<float> prev_neg_slope;
    Buffer<float> prev_pos_slope;
    Buffer<int> prev_angle;
    Buffer<int> prev_center;
    Buffer<float> blue_horizontal_times;
    Buffer<float> blue_line_slopes;  // 不会检查高度，斜率满足小于0.5，只是最长
    Interpolator interpolator;
    cv::Mat frame;
    vector<cv::Vec4i> lines_raw;                                  // 存储检测到的白色车道线段
    vector<cv::Vec4i> blue_lines_raw;                             // 存储检测到的蓝色线段
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
    void laserCallback(const sensor_msgs::LaserScan::ConstPtr &msg);
    float findFrontDistance(vector<float> ranges);
    void visualizeLines(const vector<cv::Vec4i> &lines, int level, int type);
    void visualizeLidar(vector<float> distances);
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
    bool video_feed_back = false;

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

class Park : public Ability {
private:
    bool is_running_ = false;
    bool video_feed_back = false;
    bool exit_park = true;  // 满足条件后是否退出，测试使用
    bool third_stage_begin = false;
    bool visualize_lidar;
    int third_stage_begin_times = 0;  // 一个用于统计第多少次没有找到车库后边缘线的变量
    float min_distance = 0.24;
    ros::Subscriber sub_;
    cv::Mat frame;
    string camera = "back";  // 视频流的方向
    int frame_height = 480;
    int frame_width = 720;
    int handle_rate_ = 20;
    int target_index = 0;  // 0 代表左侧车库，1代表右侧车库
    int first_stage_param = -2;
    int offset_x = 0;  // 用于矫正前后摄像头中心
    float offset_top_ratio = 0.3;
    float offset_bottom_ratio = 0.1;
    int offset_top = static_cast<int>(offset_top_ratio * frame_height);
    int offset_bottom = static_cast<int>(offset_bottom_ratio * frame_height);
    cv::Point target_center;
    std::vector<std::vector<cv::Point>> contours;
    std::vector<tuple<int, int, cv::Point2i>> blueContours;  // 边缘集合的下标、面积、中心点
    float lowerFraction = 0.55;
    int lowerHeight = static_cast<int>(frame_height * lowerFraction);
    int upperHeight = frame_height - lowerHeight;
    vector<cv::Vec4i> lines_raw;  // 存储检测到的白色车道线段
    vector<tuple<cv::Vec4i, float, float, cv::Vec2i, int>>
        laneLines;  // 车库底线，线段，长度，斜率 , 中点， 和下边界的交点
    vector<tuple<cv::Vec4i, float, float, cv::Vec2i, int>>
        bottomLines;  // 车库底线，线段，长度，斜率 , 中点， 和下边界的交点
    tuple<cv::Vec4i, float, float, cv::Vec2i, int> rightLane;  // 右侧车道线
    tuple<cv::Vec4i, float, float, cv::Vec2i, int> leftLane;   // 左侧车道线
    Buffer<int> right_point;                                   // 储存右车道和下边缘的交点
    Buffer<int> left_point;                                    // 储存左车道和下边缘的交点
    Buffer<int> target_x;                                      // 储存目标标志牌的中线点x坐标
    Buffer<int> prev_angle;                                    // 储存历史旋转角度
    bool second_stage = false;             // 是否进入第二阶段（距离足够近，但未进入车库）
    Buffer<float> left_lane_found_times;   // 用来判断最近的n次中有几次是查找到左车道线的。
    Buffer<float> right_lane_found_times;  // 用来判断最近的n次中有几次是查找到右车道线的。
    Interpolator interpolator;
    int bottom_line_found_times = 0;
    int window_peroid = 0;
    float window_peroid_times = 1.2;
    bool third_stage = false;
    int times_before_end = 6;  // 默认是0.5s实际上由，frame_rate和speed决定。
    int min_bottom_length = 60;
    int frame_rate_ = 10;

public:
    Park(int remain_time, ros::NodeHandle nh);
    Park(int remain_time, ros::NodeHandle &nh, ParkInitParams params);

    float calculateSlope(const cv::Vec4i &line);
    void moveToPlace();
    void getContour();
    void contourPreprocess();
    void linePreprocess();
    void checkBottomLine();
    void getIntersection();
    void getContourCenter();
    void getLines();
    float findDistance(vector<float> ranges);
    void visualizeLidar(vector<float> distances);
    void laserCallback(const sensor_msgs::LaserScan::ConstPtr &msg);
    void imageCallback(const sensor_msgs::ImageConstPtr &msg);
    void run();
    void stop();
};