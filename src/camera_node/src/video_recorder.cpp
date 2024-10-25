#include <condition_variable>
#include <cv_bridge/cv_bridge.h>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <string>
#include <thread>

#define TAG "[video recorder]"

using namespace std;

class CameraNode {
private:
    cv::VideoCapture cap_;
    ros::NodeHandle nh_;
    cv::VideoWriter video;
    int fps = 30;
    string output_path_;
    int total_frame{};
    mutex queue_mutex;
    queue<cv::Mat> frame_queue;
    condition_variable frame_cv;
    bool is_running = true;
    thread write_thread;

public:
    CameraNode(ros::NodeHandle &nh) : nh_(nh) {
        int camera_port = 0;
        nh.getParam("camera_port", camera_port);
        cap_.open("/dev/camera" + to_string(camera_port));  // 打开摄像头

        cap_.set(cv::CAP_PROP_FRAME_WIDTH, 1920);   // 设置宽度为720
        cap_.set(cv::CAP_PROP_FRAME_HEIGHT, 1080);  // 设置高度为480
        cap_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));

        nh_.param<string>("output_video_path", output_path_, "");

        ROS_INFO("%s video path: %s", TAG, output_path_.c_str());
        int codec = cv::VideoWriter::fourcc('M', 'J', 'P', 'G');

        int frame_width = static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_WIDTH));
        int frame_height = static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_HEIGHT));

        ROS_INFO(TAG "frame width: %d",frame_width);
        ROS_INFO(TAG "frame height: %d",frame_height);

        int frame_width = 720;
        int frame_height = 480;

        int fps = cap_.get(cv::CAP_PROP_FPS);
        ROS_INFO("%s capture frame rate: %d ", TAG, fps);
        cv::Size frame_size(frame_width, frame_height);
        video = cv::VideoWriter(output_path_, codec, fps, frame_size, true);
        if (!cap_.isOpened()) {
            ROS_ERROR("Failed to open the camera");
            ros::shutdown();
        }
        if (!video.isOpened()) {
            ROS_ERROR("Failed to create video");
            ros::shutdown();
        }
        ROS_INFO("%s Creating write thread ", TAG);
        write_thread = thread(&CameraNode::writeVideo, this);
        // write_thread.detach();
    }

    void writeVideo() {
        while (is_running || !frame_queue.empty()) {
            unique_lock<std::mutex> lock(queue_mutex);
            frame_cv.wait(lock, [this] { return !frame_queue.empty() || !is_running; });

            while (!frame_queue.empty()) {
                cv::Mat frame = frame_queue.front();
                frame_queue.pop();
                lock.unlock();

                // 也许会有更好的办法显示，现在就干正事吧
                cv::Mat resized_frame;
                cv::resize(frame, resized_frame, cv::Size(frame.cols / 2, frame.rows / 2)); // 将图像大小缩小为原来的一半
                cv::imshow("video writer", resized_frame);
                video.write(frame);
                total_frame++;
                ROS_INFO("%s Current Queue Length: %d ", TAG, static_cast<int>(frame_queue.size()));
                lock.lock();
            }
        }
        cout << "Writing thread exiting." << endl;
    }

    void run() {
        ROS_INFO("%s started runing ! ", TAG);
        cv::Mat frame;
        while (ros::ok()) {
            cap_ >> frame;// 捕获图像
            if (frame.empty()) {
                ROS_WARN("Empty frame received");
                return;
            }
            {
                lock_guard<std::mutex> lock(queue_mutex);
                frame_queue.push(frame.clone());
            }

            frame_cv.notify_one();
            total_frame += 1;
            ROS_INFO("%s Current frame: %d", TAG, total_frame);
            auto key = cv::waitKey(1);
            if (key == 'q') {
                stop();
                break;
            }
        }
    }

    void stop() {
        {
            lock_guard<std::mutex> lock(queue_mutex);
            is_running = false;// 停止主线程的帧捕获
        }
        ROS_INFO("%s Exting video recorder! ", TAG);
        frame_cv.notify_one();
    }

    ~CameraNode() {
        if (write_thread.joinable()) {
            write_thread.join();
        }
        cap_.release();
        video.release();
    }
};

CameraNode *recorderPtr;
void signalHandler(int signum) {
    if (recorderPtr) {
        recorderPtr->stop();
    }
    exit(signum);
}

int main(int argc, char **argv) {
    ROS_INFO("%s Used for record video in runtime! ", TAG);
    ros::init(argc, argv, "video recorder");
    ROS_INFO("%s Used for record video in runtime! ", TAG);
    cv::namedWindow("video writer", cv::WINDOW_AUTOSIZE);
    ROS_INFO("%s Used for record video in runtime! ", TAG);
    ros::NodeHandle nh;
    CameraNode video_recorder(nh);
    recorderPtr = &video_recorder;
    ROS_INFO("%s Video Recorder node init...", TAG);
    video_recorder.run();
    cv::destroyAllWindows();
    return 0;
}
