@ingroup camera 

# Camera模块
===

## main.cpp {#camera_node}
读取前后摄像头读取到的视频流并封装为 `ros topic` 发布。
有两种模式，分别是读取实时的摄像头数据和从文件中读取一段视频作为模拟.
这两种模式是由camera模块下的CmakeLists中的编译选项决定的。

```cmake
# 默认设置为 OFF
option(USE_SIMULATION "Use simulation mode" OFF)
if (CMAKE_SYSTEM_NAME STREQUAL "Linux")
    file(READ "/proc/version" VERSION_STRING)
    if (VERSION_STRING MATCHES "microsoft")
        message(STATUS "=========================")
        message(STATUS "Detected WSL environment.")
        message(STATUS "=========================")
        set(USE_SIMULATION ON)  # 在 WSL 中启用模拟模式
    elseif (VERSION_STRING MATCHES "tegra")
        message(STATUS "=========================")
        message(STATUS "Detected native Tegra environment.")
        message(STATUS "=========================")
        set(USE_SIMULATION OFF) # 在实际的 Tegra 机器中禁用模拟模式
    else()
        message(STATUS "=========================")
        message(STATUS "Detected other Linux environment.")
        message(STATUS "=========================")
        set(USE_SIMULATION OFF) # 在其他 Linux 系统中禁用模拟模式
    endif()
endif()
```

这段代码检测了是否是wsl，如果是则默认开启Simualtion、否则禁用。

如果需要修改则可以将 `if` 一段删除，只保留：
```cmake
option(USE_SIMULATION "Use simulation mode" OFF)
```

## minimal_test.cpp {#minimal_test}
最小化测试，直接使用opencv从摄像头读取数据，仅用于测试摄像头是否可用


## hsv_extrator.cpp {#hsv_extrator}
用于可视化的去筛选hsv的范围

## video_recorder.cpp {#video_recorder}
视频录制，可以直接录制一段智能车前摄像头或者后摄像头的数据并保存到文件中，可以通过 `sftp` 命令等方法从 `Jetson Nano` 上拉去下来在本地用于测试。

## detect_light.cpp {#detect_light}
检测红绿灯的代码（测试使用）


