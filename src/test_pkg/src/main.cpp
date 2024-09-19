#include <iostream>
#include <bits/stdc++.h>
#include <ros/ros.h>
#include <serial/serial.h>
 
using namespace std;
serial::Serial ser;
void USART_SEND(serial::Serial &ser, int angle, int speed);

int main()
{
    int index = 1000000;
    for (int i = 0; i < index; i++)
    {
        USART_SEND();
    }
    return 0;
}

void USART_SEND(serial::Serial &ser, int angle, int speed)
{
    cout << "[DEBUG] USART_SEND called___ angle: " << angle << " speed: " << speed << endl;
    auto *tx_buf = new uint8_t[9];
    tx_buf = "R200W2\r\n\0";
    tx_buf[7] = '\n';
    tx_buf[8] = '\0';
    ser.write(tx_buf, 8);
    delete[] tx_buf;
}