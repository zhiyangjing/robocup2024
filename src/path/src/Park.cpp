#include <iostream>
#include <path/path.h>
#include <queue>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/LaserScan.h>
#include <stack>

#define TAG " [PARK] "

Park::Park(int remain_time, ros::NodeHandle nh) : Ability(remain_time, nh) {
    right_point = Buffer<int>(5);
    left_point = Buffer<int>(5);
    target_x = Buffer<int>(5);
    prev_angle = Buffer<int>(3);
    left_lane_found_times = Buffer<float>(10, 0);
    right_lane_found_times = Buffer<float>(10, 0);

    int point_nums = 16;
    first_stage_param = -2;

    VectorXf weights(3);
    VectorXf R(point_nums);
    VectorXf C(point_nums);
    VectorXf L(point_nums);
    VectorXf A(point_nums);
    weights << 1, 1, 1;

    // 默认的一套参数，出于开发历史原因，这套参数适用于倒车时的情形
    R << 650, 650, 685, 685, 600, 560, 550, 760, 778, 760, 600, 670, 750, 710, 650, 670;
    C << 331, 334, 328, 328, 301, 288, 206, 359, 360, 355, 233, 275, 325, 400, 377, 401;
    L << 42, 48, 31, 26, -43, -70, -54, 95, 135, 154, -50, 33, 150, 40, 44, 30;
    A << 0, 0, 0, 0, 200, 150, 100, -200, -150, -100, 100, 60, 30, -100, -60, -30;

    // R << 609, 627, 612, 889, 876, 370, 370, 588, 737, 743, 559;
    // C << 363, 370, 360, 424, 444, 311, 303, 249, 470, 489, 270;
    // L << 124, 136, 113, 363, 369, -136, -128, 37, 200, 253, 30;
    // A << 0, 0, 0, -200, -200, 200, 200, 100, -100, -150, 150;

    // 创建点矩阵
    MatrixXf points(point_nums, 3);
    for (int i = 0; i < point_nums; ++i) {
        points(i, 0) = R(i);  // 第一列为 rightpoint
        points(i, 1) = C(i);  // 第二列为 centerpoint
        points(i, 2) = L(i);  // 第三列为 leftpoint
    }

    interpolator = Interpolator(points, A, weights);

    target_index = 0;  // 0 代表左侧车库, 这个初始化函数本身就是给倒车用的，

    ROS_INFO(TAG "Target index: %d", target_index);

    int exit_park_param = 1;
    nh_.getParam("exit_park", exit_park_param);
    exit_park = (exit_park_param == 1);
    ROS_INFO(TAG COLOR_MAGENTA "Exit Park %s" COLOR_RESET,
             (exit_park) ? (COLOR_GREEN "Enabled") : (COLOR_RED "Disabled"));

    nh_.getParam("camera_node/" + camera + "/frame_rate", frame_rate_);

    float time_before_exit = 0.6;
    nh_.getParam("time_before_exit_" + camera, time_before_exit);
    times_before_end = frame_rate_ * time_before_exit;
    ROS_INFO(TAG COLOR_MAGENTA "Times before exit: %d" COLOR_RESET, times_before_end);

    nh_.getParam("window_peroid_times", window_peroid_times);
    ROS_INFO(TAG COLOR_MAGENTA "Window Peroid Time %f" COLOR_RESET, window_peroid_times);

    ROS_INFO(TAG COLOR_MAGENTA "Park node started" COLOR_RESET);
}

Park::Park(int remain_time, ros::NodeHandle &nh, ParkInitParams params) : Ability(remain_time, nh) {
    right_point = Buffer<int>(5);
    left_point = Buffer<int>(5);
    target_x = Buffer<int>(5);
    prev_angle = Buffer<int>(3);
    left_lane_found_times = Buffer<float>(10, 0);
    right_lane_found_times = Buffer<float>(10, 0);

    // 控制点个数
    int point_nums = params.ref_points.cols();
    first_stage_param = params.first_stage_param;
    camera = params.camera;
    lowerFraction = params.lowerFraction;
    lowerHeight = static_cast<int>(frame_height * lowerFraction);
    upperHeight = frame_height - lowerHeight;

    // 创建点矩阵
    MatrixXf points(point_nums, 3);
    for (int i = 0; i < point_nums; ++i) {
        points(i, 0) = params.ref_points.row(0)[i];  // 第一列为 rightpoint
        points(i, 1) = params.ref_points.row(1)[i];  // 第二列为 centerpoint
        points(i, 2) = params.ref_points.row(2)[i];  // 第三列为 leftpoint
    }

    interpolator = Interpolator(points, params.ref_value, params.weights);

    string target = "left";
    nh_.getParam("target", target);
    if (target == "left") {
        target_index = 0;  // 0 代表左侧车库
    } else {
        target_index = 1;  // 1代表右侧车库
    }

    if (camera == "front") {
        offset_x = 30;
    } else {
        offset_x = 0;
    }

    ROS_INFO(TAG "Target index: %d", target_index);

    int exit_park_param = 1;
    nh_.getParam("exit_park", exit_park_param);
    exit_park = (exit_park_param == 1);
    ROS_INFO(TAG COLOR_MAGENTA "Exit Park %s" COLOR_RESET,
             (exit_park) ? (COLOR_GREEN "Enabled") : (COLOR_RED "Disabled"));

    nh_.getParam("camera_node/" + camera + "/frame_rate", frame_rate_);

    float time_before_exit = 0.6;
    nh_.getParam("time_before_exit", time_before_exit);
    times_before_end = frame_rate_ * time_before_exit;
    ROS_INFO(TAG COLOR_MAGENTA "Times before exit: %d" COLOR_RESET, times_before_end);

    nh_.getParam("window_peroid_times", window_peroid_times);
    ROS_INFO(TAG COLOR_MAGENTA "Window Peroid Time %f" COLOR_RESET, window_peroid_times);

    ROS_INFO(TAG COLOR_MAGENTA "Park node started" COLOR_RESET);
}

void Park::moveToPlace() {
    if (third_stage and exit_park) {
        if (times_before_end < 0) {
            nh_.setParam("speed", 0);
            stop();
        }
        times_before_end--;
        int angle_value = prev_angle.avg() * 0.3;
        nh_.setParam("angle", angle_value);
        ROS_INFO(TAG COLOR_YELLOW "Time before end: %d " COLOR_RESET, times_before_end);
    } else if (not second_stage) {
        int x = target_x.avg();
        int res = (x - (frame_width / 2 + offset_x)) * first_stage_param;
        cv::circle(frame, cv::Point(frame_width / 2 + offset_x, frame_height - 10), 3, cv::Scalar(0, 255, 0),
                   cv::FILLED);  // 使用 cv::FILLED 填充圆
        cv::circle(frame, cv::Point(x, frame_height - 10), 3, cv::Scalar(0, 0, 255),
                   cv::FILLED);  // 使用 cv::FILLED 填充圆
        prev_angle.push(max(min(res, 200), -200));
        int angle_value = prev_angle.avg();
        nh_.setParam("angle", angle_value);
    } else {
        VectorXf point_data(3);
        // ROS_INFO(TAG "RIGHT: %d", right_point.avg());
        // ROS_INFO(TAG "LEFT: %d", left_point.avg());
        point_data << right_point.avg(), target_x.avg(), left_point.avg();
        float res = max(min(interpolator.interpolate(point_data), 200.f), -200.f);
        prev_angle.push(static_cast<int>(res));
        int angle_value = prev_angle.avg();
        nh_.setParam("angle", angle_value);
    }

    ROS_INFO(TAG "RIGHT: %d, CENTER: %d, LEFT: %d, ANGLE: %d, STAGE: %d", right_point.avg(), target_x.avg(),
             left_point.avg(), prev_angle.avg(), static_cast<int>(second_stage));
}

void Park::getContour() {
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

    cv::Mat erosion_element = cv::getStructuringElement(
        cv::MORPH_RECT, cv::Size(2 * erosion_size + 1, 2 * erosion_size + 1), cv::Point(erosion_size, erosion_size));

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

void Park::contourPreprocess() {
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

float Park::calculateSlope(const cv::Vec4i &line) {
    float dx = line[2] - line[0];
    float dy = line[3] - line[1];
    if (dx == 0) {
        return numeric_limits<float>::infinity();  // 处理垂直线
    }
    return dy / dx;  // 计算斜率
}

void Park::linePreprocess() {
    laneLines.clear();
    bottomLines.clear();
    if (lines_raw.empty()) {
        ROS_WARN(TAG "No line to preprocess");
    } else {
        // ROS_INFO(TAG "lines count %d", (int) lines_raw.size());
        for (const auto &line : lines_raw) {
            float slope = calculateSlope(line);

            {
                cv::Point start(line[0], line[1] + (upperHeight));
                cv::Point end(line[2], line[3] + (upperHeight));
                cv::line(frame, start, end, cv::Scalar(255, 255, 0), 2);
            }

            double lineLength = cv::norm(cv::Point(line[0], line[1]) - cv::Point(line[2], line[3]));
            if (fabs(slope) < 0.2) {
                bottomLines.emplace_back(line, lineLength, slope,
                                         cv::Vec2i((line[0] + line[2]) / 2, (line[1] + line[3]) / 2 + upperHeight),
                                         0);  // 不填充于底边的交点（无用）
                ROS_INFO(TAG "slope : %f length: %f center_x:  %d center_y: %d line size:  %d ", slope, lineLength,
                         (line[0] + line[2]) / 2, (line[1] + line[3]) / 2 + upperHeight,
                         static_cast<int>(lines_raw.size()));
                continue;  // 去除横向线段，以及过于垂直的线段
            }

            int x0 = line[0], y0 = line[1], x1 = line[2], y1 = line[3];
            int intersection_pos = (lowerHeight - y0) * (x1 - x0) / (y1 - y0) + x0;

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

/**
     * @brief 检测底线是否经过识别区域下方，如果经过，则设置为第三阶段，后退固定时长然后停住。
     * 
     */
void Park::checkBottomLine() {
    if (bottomLines.empty() or not second_stage or third_stage) {
        return;
    }

    if (window_peroid > 0) {
        window_peroid--;
        ROS_INFO(TAG COLOR_YELLOW "Window Peroid: %d " COLOR_RESET, window_peroid);
        return;
    }

    sort(bottomLines.begin(), bottomLines.end(), [](const auto &a, const auto &b) { return get<1>(a) > get<1>(b); });
    auto longestLine = bottomLines[0];
    // Buffer<int> pos_y(4);
    // Buffer<int> pos_x(4);
    // for (int i = 0; i < min(static_cast<int>(bottomLines.size()), 4); i++) {
    int pos_y = get<3>(longestLine)[1];
    int pos_x = get<3>(longestLine)[0];
    // }

    // ROS_INFO(TAG "%f", get<1>(longestLine));
    if (pos_y > 445 and pos_x < (frame_width - 80) and pos_x > 80) {
        if ((get<1>(longestLine) > min_bottom_length and bottomLines.size() > 2)
            or (get<1>(longestLine) > 20 and bottomLines.size() > 4)) {
            bottom_line_found_times += 1;
            window_peroid = window_peroid_times * frame_rate_;
            if (bottom_line_found_times == 2) {
                times_before_end *= (fabs(get<2>(longestLine)) + 1);
                third_stage = true;
                ROS_INFO(TAG COLOR_MAGENTA "Stage 3 started! " COLOR_RESET);
                ROS_INFO(TAG COLOR_YELLOW "Frame rate got: %d " COLOR_RESET, frame_rate_);
                ROS_INFO(TAG COLOR_YELLOW "Time before end: %d " COLOR_RESET, times_before_end);
            }
            ROS_INFO(TAG COLOR_YELLOW "Bottom Line detected!" COLOR_RESET);
            ROS_INFO(TAG COLOR_YELLOW "Window Peroid: %d " COLOR_RESET, window_peroid);

            for (const auto &line : bottomLines) {
                ROS_INFO(TAG "slope : %f length: %f center_x:  %d center_y: %d bottomLine size:  %d ", get<2>(line),
                         get<1>(line), get<3>(line)[0], get<3>(line)[1], static_cast<int>(bottomLines.size()));
            }

            // ROS_INFO(TAG "camera_node ");
            // ROS_INFO(TAG "bottomLines size:  %d ", static_cast<int>(bottomLines.size()));
        }
    }
    // 长度大于特定最小值，并且处于屏幕下方
}

void Park::getIntersection() {
    int c_x = target_x.avg();
    sort(laneLines.begin(), laneLines.end(),
         [c_x](auto const &a, auto const &b) { return abs(get<3>(a)[0] - c_x) < abs(get<3>(b)[0] - c_x); });
    bool right_lane_found = false;
    bool left_lane_found = false;
    for (const auto &Lane : laneLines) {
        auto line = get<0>(Lane);
        auto length = get<1>(Lane);
        auto slope = get<2>(Lane);
        auto center_x = get<3>(Lane)[0];
        auto intersection_pos = get<4>(Lane);
        if (not right_lane_found and (slope > 0 or slope < -20) and center_x > c_x and intersection_pos < 920) {
            cv::Point start(line[0], line[1] + (upperHeight));
            cv::Point end(line[2], line[3] + (upperHeight));
            cv::line(frame, start, end, cv::Scalar(255, 0, 0), 2);
            cv::circle(frame, get<3>(Lane), 5, cv::Scalar(0, 255, 0), -1);
            rightLane = Lane;
            right_lane_found = true;
            right_point.push(get<4>(Lane));
        } else if (not left_lane_found and length > 50 and (slope < 0 or slope > 20) and center_x < c_x
                   and intersection_pos > -200) {
            cv::Point start(line[0], line[1] + (upperHeight));
            cv::Point end(line[2], line[3] + (upperHeight));
            cv::line(frame, start, end, cv::Scalar(0, 255, 0), 2);
            cv::circle(frame, get<3>(Lane), 5, cv::Scalar(255, 0, 0), -1);
            leftLane = Lane;
            left_lane_found = true;
            left_point.push(get<4>(Lane));
        }
    }

    left_lane_found_times.push(static_cast<float>(left_lane_found));
    right_lane_found_times.push(static_cast<float>(right_lane_found));

    if (left_lane_found_times.avg() >= 0.7 and right_lane_found_times.avg() >= 0.7 and not second_stage) {
        second_stage = true;
        nh_.setParam("speed", 1);
        ROS_INFO(TAG COLOR_MAGENTA "Stage 2 started! " COLOR_RESET);
        ROS_INFO(TAG COLOR_MAGENTA "Speed set to 1" COLOR_RESET);
    }
}

void Park::getContourCenter() {
    sort(blueContours.begin(), blueContours.end(), [](auto const &a, auto const &b) { return get<1>(a) > get<1>(b); });

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
            // cout << COLOR_RED "not valid: " << COLOR_RESET << ratio << endl;
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
        target_center = cv::Point(frame_width / 2 + offset_x, frame_height / 2);
    }
    target_x.push(target_center.x);
}

void Park::getLines() {
    // 计算下部分的高度，根据给定的比例
    cv::Rect lowerPartRect(0, upperHeight, frame_width, lowerHeight);
    cv::Mat lowerPart = frame(lowerPartRect);  // 提取下部分图像

    cv::Mat hsv_frame;
    cv::cvtColor(lowerPart, hsv_frame, cv::COLOR_BGR2HSV);

    // 定义白色的HSV范围
    cv::Scalar lowerWhite(0, 0, 165);     // 白色下限
    cv::Scalar upperWhite(180, 35, 255);  // 白色上限

    // 创建白色区域的掩码
    cv::Mat mask;
    cv::inRange(hsv_frame, lowerWhite, upperWhite, mask);
    // cv::imshow("white", mask);

    int erosion_size = 1;   // 腐蚀结构元素的大小
    int dilation_size = 1;  // 膨胀结构元素的大小

    cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2 * erosion_size + 1, 2 * erosion_size + 1),
                                                cv::Point(erosion_size, erosion_size));

    cv::erode(mask, mask, element);   // 腐蚀
    cv::dilate(mask, mask, element);  // 膨胀
    // cv::imshow("iamge", mask);

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
    // cv::imshow("mask",mask);

    cv::imshow("contour", contourImage);

    // 使用 HoughLinesP 检测线段
    cv::dilate(contourImage, contourImage, element);  // 膨胀
    cv::HoughLinesP(contourImage, lines_raw, 1, CV_PI / 180, 50, 35, 25);
    // cv::HoughLinesP(contourImage, lines_raw, 2, CV_PI / 180, 30, 20, 20);
    // 在获得中点之后再搜索可用线段。
}

void Park::imageCallback(const sensor_msgs::ImageConstPtr &msg) {
    frame = cv_bridge::toCvShare(msg, "bgr8")->image;
    cv::resize(frame, frame, cv::Size(frame_width, frame_height));
    if (frame.empty()) {
        ROS_WARN("Empty frame received");
        return;
    }

    // 流水线处理，顺序调整得检查依赖关系
    getContour();
    contourPreprocess();
    getContourCenter();  // 在获得中点之后再搜索可用线段。
    getLines();
    linePreprocess();
    getIntersection();  // 依赖于getLines()和getContourCenter()
    checkBottomLine();  // 依赖于getLines()
    moveToPlace();

    cv::imshow("camera_node Feed", frame);
    cv::waitKey(10);

    ros::spinOnce();  // 处理 ROS 事件
}

void Park::run() {
    ROS_INFO(TAG COLOR_GREEN "Park started to run" COLOR_RESET);
    is_running_ = true;
    sub_ = nh_.subscribe("/image_topic/" + camera, 1, &Park::imageCallback, this);

    if (camera == "back") {
        nh_.setParam("direction", std::string(1, 'S'));
    } else {
        nh_.setParam("direction", std::string(1, 'W'));
    }
    nh_.setParam("angle", 0);
    ros::Rate handle_rate(handle_rate_);  // 处理频率
    while (ros::ok() && is_running_) {
        ros::spinOnce();
        handle_rate.sleep();
    }
}

void Park::stop() {
    is_running_ = false;
    sub_.shutdown();
}

// /**
//  * @brief 这里只是一个测试用的main函数，实际上真正使用这里的功能是在main.cpp
//  *
//  */
// int main(int argc, char **argv) {
//     // 测试节点
//     ros::init(argc, argv, "trace_line");
//     ros::NodeHandle nh;
//     cv::namedWindow("camera_node Feed", cv::WINDOW_AUTOSIZE);

//     Park reverse_park(10, nh);
//     reverse_park.run();
//     cv::destroyAllWindows();
//     return 0;
// }
