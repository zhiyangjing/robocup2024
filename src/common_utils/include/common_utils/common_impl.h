#include "common.h"
#include <cstdio>
#include <cstring>
#include <iostream>
#include <ros/ros.h>
#include <stddef.h>
// 构造函数
template<typename T> Buffer<T>::Buffer() {
    buffer = nullptr;
    printf("size location: %pl\n", &size);
    printf("buffer location: %p\n", buffer);
}

/**
 * @brief 创建Buffer但是不填充，其中的size开始为0，直到有pus进的元素
 * 
 * @tparam T 
 * @param capacity 
 */
template<typename T> Buffer<T>::Buffer(size_t capacity) : capacity(capacity), size(0), head(0) {
    buffer = new T[capacity];
    memset(buffer, 0, sizeof(T) * capacity);
    printf("buffer location: %p\n", buffer);
}

/**
 * @brief 创建Buffer同时会填充其中所有的位置，这个时候size一开始就是设置的大小（即已填满，会在计算avg的时候使用这个size）
 * 
 * @tparam T 
 * @param capacity 
 * @param default_value 
 */
template<typename T> Buffer<T>::Buffer(size_t capacity, T default_value) : capacity(capacity), size(0), head(0) {
    buffer = new T[capacity];
    for (size_t i = 0; i < capacity; i++) {
        buffer[i] = default_value;
    }
    size = 5;
    printf("buffer location: %p\n", buffer);
}

// 拷贝构造函数
template<typename T> Buffer<T>::Buffer(const Buffer<T> &other) {
    capacity = other.capacity;
    size = other.size;
    head = other.head;

    if (other.buffer) {
        buffer = new T[capacity];
        memcpy(buffer, other.buffer, sizeof(T) * capacity);
    } else {
        buffer = nullptr;
    }
    printf("Copy constructor called - buffer location: %p\n", buffer);
}

template<typename T> T Buffer<T>::avg() {
    T sum = 0;
    if (size == 0) {
        return 0;
    }
    for (int i = 0; i < size; i++) {
        sum += buffer[(head + i) % capacity];
    }
    return sum / static_cast<int>(size);
}

// 赋值运算符重载
template<typename T> Buffer<T> &Buffer<T>::operator=(const Buffer<T> &other) {
    if (this != &other) {  // 自赋值检查
        delete[] buffer;   // 释放旧内存

        // 复制新内容
        capacity = other.capacity;
        size = other.size;
        head = other.head;

        if (other.buffer) {
            buffer = new T[capacity];                            // 分配新内存
            memcpy(buffer, other.buffer, sizeof(T) * capacity);  // 深拷贝
        } else {
            buffer = nullptr;
        }
    }
    printf("Assignment operator called - buffer location: %p\n", buffer);
    return *this;  // 返回当前对象的引用
}

// 析构函数
template<typename T> Buffer<T>::~Buffer() { delete[] buffer; }

// 从头部添加元素
template<typename T> void Buffer<T>::push(const T &value) {
    head = (head - 1 + capacity) % capacity;  // 更新头部索引
    buffer[head] = value;

    if (size < capacity) {
        size++;  // 如果未满，则增加大小
    }
}

template<typename T> void Buffer<T>::push(const T &value, size_t times) {
    for (size_t i = 0; i < times; i++) {
        this->push(value);
    }
}

// 重载下标运算符
template<typename T> T &Buffer<T>::operator[](size_t index) {
    // 不需要越界检测，假设使用者会注意
    return buffer[(head + index) % capacity];
}

// 获取当前缓冲区的大小
template<typename T> size_t Buffer<T>::getSize() const { return size; }

// 打印缓冲区内容（可选）
template<typename T> void Buffer<T>::printBuffer() const {
    for (size_t i = 0; i < size; i++) {
        std::cout << buffer[(head + i) % capacity] << " ";
    }
    std::cout << std::endl;
}