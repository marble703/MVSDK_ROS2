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
        std::string data_dirname = "./",
        std::shared_ptr<cv::Mat> frame_ptr = nullptr
    );
    ~Camera();

    /**
     * @brief 初始化相机
     * @param try_reinit_time 重试连接次数
     * @param wait_init_time 重试等待时间
     * @param force 是否无视重初始化警告
     * @return 初始化成功
     */
    bool init(
        int try_reinit_time = 10,
        int wait_init_time = 500,
        bool force = false
    );

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

    /**
     * @brief 设置曝光时间
     * @param exposure_time 曝光时间, 单位微妙
     * @return 设置成功
     */
    bool setExposureTime(double exposure_time = -1);

    /**
     * @brief 打印相机设备信息
     * @param info 相机设备信息结构体
     */
    void PrintCameraDeviceInfo(const tSdkCameraDevInfo* info);

    /**
     * @brief 设置相机名称
     * @param name 相机名称，不得超出 32 字节
     */
    void SetCameraName(std::string name);

    /**
     * @brief 读取相机名称
     */
    std::string ReadCameraName();

    /**
     * @brief 设置相机数据目录
     * @param data_dirname 目录
     * @return 成功 
     */
    bool SetCameraDataDirectory(std::string data_dirname);

    // 释放相机资源，一般不需要手动调用，析构函数会自动调用
    void release();

private:
    // 监控是否重初始化的变量
    bool init_tag;

    // 相机启动信息
    int iCameraCounts;
    int iStatus;
    tSdkCameraDevInfo tCameraEnumList;
    int hCamera;
    tSdkCameraCapbility tCapability; //设备描述信息
    tSdkFrameHead sFrameInfo;
    BYTE* pbyBuffer;
    unsigned char* g_pRgbBuffer; //处理后数据缓存区

    // 相机配置文件路径
    std::string camera_config_path_;
    // 相机数据文件的存储目录
    std::string data_dirname_;
    // 相机名称
    std::string camera_name_;

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

    // 互斥锁，保护获取图像操作
    std::mutex mtx_getFrame;
};