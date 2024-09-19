#include <iostream>
#include <bits/stdc++.h>
#include <ros/ros.h>
#include <serial/serial.h>

using namespace std;
serial::Serial ser;
void USART_SEND(serial::Serial &ser_loc, int angle, int speed);

int main()
{
    ser.setBaudrate(115200);
    ser.setPort("/dev/ttyUSB0");
    serial::Timeout to = serial::Timeout::simpleTimeout(3000);
    ser.setTimeout(to);
    ser.open();

    if (ser.isOpen())
    {
        int index = 1000000;
        for (int i = 0; i < index; i++)
        {
            int angle = 0, speed = 0;
            ros::param::get("angle", angle);
            ros::param::get("speed", speed);
            USART_SEND(ser, angle, speed);
        }
    }
    ros::Rate looprate(500);
    while (ros::ok())
    {
        ros::spinOnce;
        looprate.sleep();
    }
    return 0;
}

void USART_SEND(serial::Serial &ser_loc, int angle, int speed)
{
    cout << "[DEBUG] USART_SEND called___ angle: " << angle << " speed: " << speed << endl;
    // auto *tx_buf = new uint8_t[9];
    // tx_buf = (uint8_t *)"R200W2\r\n\0";
    // const char *tx_buf = "R200W2\r\n\0";
    char tx_buf[9];
    sprintf(tx_buf,"R%03dW%d",angle,speed);
    cout << "[DEBUG] tx_buf content" << tx_buf << endl;
    ser_loc.write(reinterpret_cast<const uint8_t *>(tx_buf), 8);
}