#pragma once

// 相机SDK的API头文件
#include "CameraApi.h" // IWYU pragma: keep (防止 clangd 静态分析报错)

#include <opencv2/opencv.hpp>

#include <iostream>
#include <mutex>
#include <thread>

class Camera {
public:
    enum GetFrameStatus { SUCCESS = 0, LAST, TIMEOUT, LOCKED, ERROR };

    Camera(
        std::string camera_config_path = "./config/camera.config",
        double exposure_time = -1
    );
    ~Camera();

    // 初始化相机
    bool init();

    // 设置曝光时间
    bool setExposureTime(double exposure_time = -1);

    // 获取图像
    cv::Mat getFrame();

    // 释放相机资源，一般不需要手动调用，析构函数会自动调用
    void release();

    // 互斥锁，保护获取图像操作
    std::mutex mtx_getFrame;

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
};