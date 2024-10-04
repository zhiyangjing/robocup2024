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

class VideoRecorder {
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

public:
    VideoRecorder(ros::NodeHandle &nh) : nh_(nh) {
        cap_.open(0);// 打开摄像头
        nh_.param<string>("output_video_path", output_path_, "");
        ROS_INFO("%s video path: %s", TAG, output_path_.c_str());
        int codec = cv::VideoWriter::fourcc('M', 'J', 'P', 'G');
        int frame_width = static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_WIDTH));
        int frame_height = static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_HEIGHT));
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
        std::thread write_thread(&VideoRecorder::writeVideo, this);
        write_thread.detach();
    }

    void writeVideo() {
        while (is_running) {
            unique_lock<std::mutex> lock(queue_mutex);
            frame_cv.wait(lock, [this] { return !frame_queue.empty() || !is_running; });

            while (!frame_queue.empty()) {
                cv::Mat frame = frame_queue.front();
                frame_queue.pop();
                lock.unlock();

                video.write(frame);
                total_frame++;
                ROS_INFO("%s Current Queue Length: %d ",TAG, static_cast<int>(frame_queue.size()));
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
            cv::imshow("video writer", frame);
            auto key = cv::waitKey(1);
            if (key == 'q') {
                ROS_INFO("%s Exting video recorder! ", TAG);
                ros::shutdown();
                break;
            }
        }
    }
    ~VideoRecorder() {
        cap_.release();
        video.release();
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "video recorder");
    cv::namedWindow("video writer", cv::WINDOW_AUTOSIZE);
    ros::NodeHandle nh;
    VideoRecorder video_recorder(nh);
    ROS_INFO("%s Video Recorder node init...", TAG);
    video_recorder.run();
    cv::destroyAllWindows();
    return 0;
}
