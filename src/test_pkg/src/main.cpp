#include <iostream>
#include <ros/ros.h>
#include <serial/serial.h>

using namespace std;
serial::Serial ser;
void USART_SEND(serial::Serial &ser_loc, int angle, int speed);

int main()
{
    // 设置串口参数
    ser.setBaudrate(115200);
    ser.setPort("/dev/ttyUSB0");
    serial::Timeout to = serial::Timeout::simpleTimeout(3000);
    ser.setTimeout(to);

    ser.open();

    if (ser.isOpen())
    {
        // 设置主循环的频率，比如50Hz
        ros::Rate looprate(50);

        // 主循环，持续执行，直到ros::ok()返回false
        while (ros::ok())
        {
            int angle = 0, speed = 0;

            // 动态获取参数
            ros::param::get("angle", angle);
            ros::param::get("speed", speed);

            // 发送串口信息
            USART_SEND(ser, angle, speed);

            // 处理ROS回调并控制循环频率
            ros::spinOnce();
            looprate.sleep();
        }
    }
    else
    {
        cout << "Failed to open serial port!" << endl;
    }

    return 0;
}

void USART_SEND(serial::Serial &ser_loc, int angle, int speed)
{
    cout << "[DEBUG] USART_SEND called___ angle: " << angle << " speed: " << speed << endl;

    // 格式化字符串
    char tx_buf[9];
    sprintf(tx_buf,"R%03dW%d",angle,speed);

    // 输出调试信息
    cout << "[DEBUG] tx_buf content: " << tx_buf << endl;

    // 发送串口数据
    ser_loc.write(reinterpret_cast<const uint8_t *>(tx_buf), 8);
}