// common_utils/include/common_utils/traffic_light.h

#ifndef TRAFFIC_LIGHT_H
#define TRAFFIC_LIGHT_H

enum TrafficLight {
    RED = 0,   // 红灯
    GREEN = 1  // 绿灯
};

#define COLOR_RESET   "\033[0m"
#define COLOR_RED     "\033[31m"
#define COLOR_GREEN   "\033[32m"
#define COLOR_YELLOW  "\033[33m"
#define COLOR_BLUE    "\033[34m"


#endif // TRAFFIC_LIGHT_H
