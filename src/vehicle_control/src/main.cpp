#include <iostream>
#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>

#define TAG "[vehicle_control]"

using namespace std;
serial::Serial ser;

void USART_SEND(serial::Serial &ser_loc, int angle, int speed, char direction);

int main(int argc, char **argv) {
    // 初始化 ROS 节点
    ros::init(argc, argv, "vehicle_control");

    // 创建 NodeHandle，用来与 ROS 通信
    ros::NodeHandle nh;

    int serial_port_vctrl = 1;
    nh.getParam("serial_port_vctrl", serial_port_vctrl);
    // 设置串口参数
    ser.setBaudrate(115200);
    ser.setPort("/dev/sym_ttyUSB" + to_string(serial_port_vctrl));
    serial::Timeout to = serial::Timeout::simpleTimeout(3000);
    ser.setTimeout(to);

    ser.open();

    if (ser.isOpen()) {
        // 设置主循环的频率，比如50Hz
        ros::Rate looprate(15);

        // 主循环，持续执行，直到ros::ok()返回false
        int angle = 0, speed = 0;
        string direction;  // 默认方向
        while (ros::ok()) {

            // 动态获取参数
            nh.getParam("angle", angle);
            nh.getParam("speed", speed);
            nh.getParam("direction", direction);
            char dir_char = direction.empty() ? 'W' : direction[0];  // 默认 W

            // 发送串口信息
            USART_SEND(ser, angle, speed, dir_char);

            // 处理ROS回调并控制循环频率
            ros::spinOnce();
            looprate.sleep();
        }
    } else {
        cout << "Failed to open serial port!" << endl;
    }

    USART_SEND(ser, 0, 0, 'W');
    return 0;
}

void USART_SEND(serial::Serial &ser_loc, int angle, int speed, char direction) {
    ROS_DEBUG("[DEBUG] USART_SEND called angle:%d speed %d direction: %c", angle, speed, direction);

    // 格式化字符串
    char tx_buf[9];
    char side = (angle >= 0) ? 'R' : 'L';  // 根据angle确定side
    int abs_angle = abs(angle);            // 取绝对值

    sprintf(tx_buf, "%c%03d%c%d\r\n", side, abs_angle, direction, speed);

    // 输出调试信息
    ROS_INFO("%s x_buf content: %s", TAG, tx_buf);

    // 发送串口数据
    ser_loc.write(reinterpret_cast<const uint8_t *>(tx_buf), sizeof(tx_buf) - 1);  // 不发送结束符
    if (ser_loc.available()) {                                                     // 检查是否有数据可读
        std::string data = ser_loc.readline();                                     // 读取一行数据
        // std_msgs::String msg;
        // msg.data = data;// 将数据存入消息

        ROS_INFO("%s Received: %s", TAG, data.c_str());  // 打印接收到的数据
    }
}
