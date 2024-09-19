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
            USART_SEND(ser, 100, 3);
        }
    }
    ros::Rate looprate(50);
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
    const char *tx_buf = "R200W2\r\n\0";
    ser_loc.write(reinterpret_cast<const uint8_t *>(tx_buf), 8);
    delete[] tx_buf;
}