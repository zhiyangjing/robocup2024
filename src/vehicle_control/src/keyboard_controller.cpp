#include <fcntl.h>
#include <ros/ros.h>
#include <std_msgs/Char.h>
#include <std_msgs/Int32.h>
#include <termios.h>
#include <unistd.h>

char getch() {
    struct termios oldt, newt;
    char ch;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    ch = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    return ch;
}

class KeyboardController {
public:
    KeyboardController(ros::NodeHandle &nh) : nh_(nh) {
        ROS_INFO("Press 'W' to go forward, 'S' to go backward, 'A' to turn left, 'D' to turn right.");
        ROS_INFO("Press numeric keys (0-5) to set speed.");
    }

    void run() {
        while (ros::ok()) {
            char key = getch();
            handleKeyPress(key);
            ros::spinOnce();
        }
    }

private:
    ros::NodeHandle nh_;
    char direction_ = 'W';// 初始方向
    int speed_ = 0;       // 初始速度
    int angle_ = 0;       // 初始角度

    void handleKeyPress(char key) {
        if (key == 'W' || key == 'S') {
            direction_ = key;
            nh_.setParam("direction", direction_);
            ROS_INFO("Direction set to: %c", direction_);
        } else if (key >= '0' && key <= '5') {
            speed_ = key - '0';
            nh_.setParam("speed", speed_);
            ROS_INFO("Speed set to: %d", speed_);
        } else if (key == 'A') {
            if (angle_ - 20 >= -200) {
                angle_ -= 20;
                nh_.setParam("angle", angle_);
                ROS_INFO("Angle decreased to: %d", angle_);
            } 
        } else if (key == 'D') {
            if (angle_ + 20 <= 200) {
                angle_ += 20;
                nh_.setParam("angle", angle_);
                ROS_INFO("Angle increased to: %d", angle_);
            } 
        }
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "keyboard_controller_node");
    ros::NodeHandle nh;

    KeyboardController controller(nh);
    controller.run();

    return 0;
}
