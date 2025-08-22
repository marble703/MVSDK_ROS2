#pragma once

// 相机SDK的API头文件
#include "CameraApi.h" // IWYU pragma: keep (防止 clangd 静态分析报错)

#include <opencv2/opencv.hpp>

#include <iostream>
#include <mutex>
#include <thread>

namespace mindvision {
enum ReadFrameStatus {
    SUCCESS = 0, // 成功
    LAST,        // 使用上一帧(由于当次未正确读取)
    TIMEOUT,     // 相机读取超时(未启用)
    LOCKED,      // 相机对象缓冲区被锁定
    UNINIT,      // 相机未初始化
    UNKNOWN      // 未知错误
};
}

class Camera {
public:
    Camera(
        std::string camera_config_path = "./config/camera.config",
        std::shared_ptr<cv::Mat> frame_ptr = nullptr,
        double exposure_time = -1
    );
    ~Camera();

    // 初始化相机
    bool init();

    // 设置曝光时间
    bool setExposureTime(double exposure_time = -1);

    /**
     * @brief 读取一帧图像到类内缓存
     * 
     * @param status 状态
     */
    void readFrame(std::shared_ptr<int> status = nullptr);

    /**
     * @brief 从类内缓存获取一帧图像
     * @return 返回获取到的图像
     * @note 在从相机获取失败时会返回上一帧作为替代
     */
    cv::Mat getFrame();

    /**
     * @brief 从类内缓存获取一帧图像
     * @param frame_ptr 存储获取到的图像
     * @note 在从相机获取失败时会返回上一帧作为替代
     */
    void getFrame(std::shared_ptr<cv::Mat> frame_ptr);

    /**
     * @brief 获取直接指向类内缓存的指针
     * 
     * @note 为防止外部修改缓存，返回值只读
     */
    std::shared_ptr<const cv::Mat> getFramePtr();

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

    // 相机配置文件路径
    std::string camera_config_path_;

    // 重初始化尝试次数
    int reopen_count_;

    // 图像信息
    cv::Mat image;
    std::shared_ptr<cv::Mat> frame_ptr_;
    std::chrono::time_point<std::chrono::high_resolution_clock>
        last_frame_time_;
    int channel;
    // 曝光时间
    double exposure_time_;
};