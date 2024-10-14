#ifndef TRAFFIC_LIGHT_H
#define TRAFFIC_LIGHT_H
#include <Eigen/Dense>

using Eigen::MatrixXf;
using Eigen::VectorXf;
using Eigen::Vector2f;

enum TrafficLight {
    RED = 0, // 红灯
    GREEN = 1// 绿灯
};

#define COLOR_RESET "\033[0m"
#define COLOR_BLACK "\033[30m"
#define COLOR_RED "\033[31m"
#define COLOR_GREEN "\033[32m"
#define COLOR_YELLOW "\033[33m"
#define COLOR_BLUE "\033[34m"
#define COLOR_MAGENTA "\033[35m"
#define COLOR_CYAN "\033[36m"
#define COLOR_WHITE "\033[37m"

template<typename T> class Buffer {
public:
    explicit Buffer(size_t capacity);// 指定大小的构造函数
    explicit Buffer();
    ~Buffer();                                   // 析构函数
    Buffer<T> &operator=(const Buffer<T> &other);// 赋值运算符重载
    Buffer(const Buffer<T> &other);

    void push(const T &value);  // 从头部添加元素
    T &operator[](size_t index);// 重载下标运算符
    T avg();
    size_t getSize() const;  // 获取当前缓冲区的大小
    void printBuffer() const;// 打印缓冲区内容（可选）

private:
    T *buffer;      // 用于存储元素的动态数组
    size_t capacity;// 缓冲区的最大容量
    size_t size;    // 当前元素数量
    size_t head;    // 指向最新元素的索引
};

class Interpolator {
private:
    MatrixXf points;
    VectorXf values;
public:
    Interpolator();   
    Interpolator(const MatrixXf &points, const VectorXf &values);
    Interpolator& operator=(const Interpolator& other);
    float interpolate(const VectorXf point);
};

#include "common_impl.h"
#endif// TRAFFIC_LIGHT_H
