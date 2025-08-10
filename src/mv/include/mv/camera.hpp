#pragma once

#include "CameraApi.h" // IWYU pragma: keep 相机SDK的API头文件
#include <memory>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>

#include <mutex>

class Camera {
public:
    Camera(
        std::string camera_config_path = "./config/camera.config",
        double exposure_time = -1
    );
    ~Camera();

    // 初始化相机
    bool init();

    // 设置曝光时间
    void set_exposure_time(double exposure_time = -1);

    // 获取图像
    cv::Mat get_frame();

    // 检查锁状态
    bool is_locked();

    // 释放相机资源，一般不需要手动调用，析构函数会自动调用
    void release();

private:
    // 监控是否重初始化的变量
    bool init_tag;

    // 相机启动信息
    int iCameraCounts = 1;
    int iStatus = -1;
    tSdkCameraDevInfo tCameraEnumList;
    int hCamera;
    tSdkCameraCapbility tCapability; //设备描述信息
    tSdkFrameHead sFrameInfo;
    BYTE* pbyBuffer;
    unsigned char* g_pRgbBuffer; //处理后数据缓存区

    // 重初始化尝试次数
    int reopen_count_;

    // 图像信息
    cv::Mat image;
    int channel;

    // 相机配置文件路径
    std::string camera_config_path_;

    // 曝光时间
    double exposure_time_;

    // 图片获取互斥锁
    std::mutex mtx_get_frame;
};