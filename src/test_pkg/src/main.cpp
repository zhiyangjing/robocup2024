#include <iostream>
#include <bits/stdc++.h>
#include <ros/ros.h>
#include <serial/serial.h>
 
using namespace std;
serial::Serial ser;
void USART_SEND(serial::Serial &ser, int angle, int speed);

int main()
{
    ser.setBaudrate(115200);
    ser.setPort("/dev/ttyUSB1");
    serial::Timeout to = serial::Timeout::simpleTimeout(3000);
    ser.setTimeout(to);
    ser.open();
    int index = 1000000;
    for (int i = 0; i < index; i++)
    {
        USART_SEND(ser,100,3);
    }
    return 0;
}

void USART_SEND(serial::Serial &ser, int angle, int speed)
{
    cout << "[DEBUG] USART_SEND called___ angle: " << angle << " speed: " << speed << endl;
    const auto *tx_buf = new uint8_t[9];
    tx_buf = "R200W2\r\n\0";
    ser.write(tx_buf, 8);
    delete[] tx_buf;
}