#include <mv/camera.hpp>

Camera::Camera(std::string camera_config_path, double exposure_time):
    camera_config_path_(camera_config_path) {
    this->exposure_time_ = exposure_time;

    this->init_tag = false;
    this->iCameraCounts = 1;
    this->iStatus = -1;

    this->image = cv::Mat();
    this->channel = 3;
}

Camera::~Camera() {
    this->release();
}

bool Camera::init() {
    // 避免重初始化
    if (!this->init_tag) {
        std::cout << "Camera init start" << std::endl;
    } else {
        std::cout << "Tried to reinitialize camera!" << std::endl;
        return true;
    }

    // 初始化 mv SDK
    // TODO: 设置全局初始化检查，这个只需要初始化一次
    CameraSdkInit(1);
    // 枚举设备，建立设备列表
    iStatus = CameraEnumerateDevice(&tCameraEnumList, &iCameraCounts);
    std::cout << "state = " << iStatus << std::endl;
    std::cout << "count = " << iCameraCounts << std::endl;
    // 如果没有连接设备
    if (iCameraCounts == 0) {
        bool flag = false;
        int fail_count = 1;
        // 重试10次，每次间隔500ms
        while (fail_count < 10) {
            std::cerr << "No camera detected! Retry " << fail_count << " times"
                      << std::endl;

            CameraSdkInit(1);
            iStatus = CameraEnumerateDevice(&tCameraEnumList, &iCameraCounts);
            if (iCameraCounts != 0) {
                std::cout << "Camera detected!" << std::endl;
                flag = true;
                break;
            }
            fail_count++;
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
        if (flag == false) {
            std::cerr << "No camera detected!(>_<)!" << std::endl;
            exit(-1);
        }
    }

    // 相机初始化。初始化成功后，才能调用任何其他相机相关的操作接口
    iStatus = CameraInit(&tCameraEnumList, -1, -1, &hCamera);
    CameraSetAeState(hCamera, false);

    // 初始化失败
    std::cout << "state = " << iStatus << std::endl;
    if (iStatus != CAMERA_STATUS_SUCCESS) {
        std::cerr << "Camera init failed!(>_<)!" << std::endl;
        // exit(-1);
    }

    // 获得相机的特性描述结构体。该结构体中包含了相机可设置的各种参数的范围信息。决定了相关函数的参数
    CameraGetCapability(hCamera, &tCapability);
    g_pRgbBuffer = (unsigned char*)malloc(
        tCapability.sResolutionRange.iHeightMax
        * tCapability.sResolutionRange.iWidthMax * 3
    );

    /*  
    让SDK进入工作模式，开始接收来自相机发送的图像
    数据。如果当前相机是触发模式，则需要接收到
    触发帧以后才会更新图像。    
    */
    // 识别相机模式
    CameraPlay(hCamera);
    if (tCapability.sIspCapacity.bMonoSensor) {
        channel = 1;
        std::cerr << "Camera is mono sensor!(>_<)!" << std::endl;
        exit(-1);
    } else {
        channel = 3;
        CameraSetIspOutFormat(hCamera, CAMERA_MEDIA_TYPE_BGR8);
    }
    // 用配置文件导入相机参数
    iStatus = CameraReadParameterFromFile(hCamera, camera_config_path_.data());
    // 万一文件位置错了
    if (iStatus != CAMERA_STATUS_SUCCESS) {
        std::cerr << "Couldn't read camera parameter from file!!!" << std::endl;
        exit(-1);
    } else {
        std::cout << "Camera parameter read success" << std::endl;
    }
    this->init_tag = true;
    this->image = this->get_frame();
    if (this->image.empty()) {
        std::cerr << "First try to get frame failed!" << std::endl;
        return false;
    } else {
        std::cout << "First try to get frame success" << std::endl;
    }
    std::cout << "Camera init success" << std::endl;
    return true;
}

// TODO: 读一下SDK, 写异常处理
bool Camera::set_exposure_time(double exposure_time) {
    if (exposure_time < 0) {
        exposure_time = this->exposure_time_;
    }
    CameraSetExposureTime(hCamera, exposure_time);
    std::cout << "Exposure time set to " << exposure_time << std::endl;
    return true;
}

void Camera::release() {
    CameraUnInit(hCamera);
    free(g_pRgbBuffer);
    std::cout << "Camera released" << std::endl;

    // TODO: 检查一下其他的，虽然感觉没用
    // 重置相机状态
    this->init_tag = false;
    this->iCameraCounts = 1;
    this->iStatus = -1;
    this->image = cv::Mat();
}
cv::Mat Camera::get_frame() {
    // 如果相机未初始化
    if (this->init_tag == false) {
        std::cerr << "Camera not initialized!" << std::endl;
        return cv::Mat();
    }

    // 如果相机获取图像时被锁
    if (this->mtx_get_frame.try_lock() == false) {
        std::cerr << "Camera get frame is locked!" << std::endl;
        return this->image;
    }

    // 正常获取到图像
    if (CameraGetImageBuffer(hCamera, &sFrameInfo, &pbyBuffer, 1)
        == CAMERA_STATUS_SUCCESS)
    {
        // 处理图像
        CameraImageProcess(hCamera, pbyBuffer, g_pRgbBuffer, &sFrameInfo);
        std::cout << "Camera get frame success" << std::endl;

        cv::Mat(sFrameInfo.iHeight, sFrameInfo.iWidth, CV_8UC3, g_pRgbBuffer)
            .copyTo(this->image);

        // 如果你认为相机节点无法运行，可以启用这个代码看一下
        // cv::imshow("Camera Image", this->image);
        // cv::waitKey(1);

        // 在成功调用 CameraGetImageBuffer 后，必须调用 CameraReleaseImageBuffer 来释放获得的 buffer 。
        // 否则再次调用 CameraGetImageBuffer 时，程序将被挂起一直阻塞
        // 直到其他线程中调用 CameraReleaseImageBuffer 来释放 buffer
        CameraReleaseImageBuffer(hCamera, pbyBuffer);

        this->mtx_get_frame.unlock();
        return this->image;
    }
    // 读取失败
    else
    {
        std::cerr << "Camera get frame failed! Tried to use last image!"
                  << std::endl;
        // 从未读取到图像
        if (this->image.empty()) {
            std::cerr << "Last image is empty, no image had been received!!!"
                      << std::endl;
        }
        // 读到过图像，退化为输出上一次读取到的图像
        // 这一般是读取频率过高导致
        else
        {
            std::cout << "Last image is not empty, using it!" << std::endl;
            std::cout << "It usually occurs when the camera is too busy."
                      << std::endl;
        }

        this->mtx_get_frame.unlock();

        return this->image;
    }
}