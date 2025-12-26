#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_srvs/srv/empty.hpp>

#include <opencv2/opencv.hpp>

#include <cv_bridge/cv_bridge.h>

#include <filesystem>

class ImageRecorderNode: public rclcpp::Node {
public:
    ImageRecorderNode():
        Node("image_recorder"),
        image_count_(0),
        is_recording_(false) {
        // 声明参数
        this->declare_parameter("output_dir", "recorded_images");
        this->declare_parameter("image_format", "jpg");
        this->declare_parameter("auto_start_recording", false);

        // 获取参数
        output_dir_ = this->get_parameter("output_dir").as_string();
        image_format_ = this->get_parameter("image_format").as_string();
        bool auto_start = this->get_parameter("auto_start_recording").as_bool();

        // 创建输出目录
        createOutputDirectory();

        // 创建订阅者
        image_subscription_ =
            this->create_subscription<sensor_msgs::msg::Image>(
                "mv/raw_image",
                rclcpp::QoS(10),
                std::bind(
                    &ImageRecorderNode::imageCallback,
                    this,
                    std::placeholders::_1
                )
            );

        // 创建服务用于控制录制开始/停止
        start_recording_service_ = this->create_service<std_srvs::srv::Empty>(
            "start_recording",
            std::bind(
                &ImageRecorderNode::startRecordingCallback,
                this,
                std::placeholders::_1,
                std::placeholders::_2
            )
        );

        stop_recording_service_ = this->create_service<std_srvs::srv::Empty>(
            "stop_recording",
            std::bind(
                &ImageRecorderNode::stopRecordingCallback,
                this,
                std::placeholders::_1,
                std::placeholders::_2
            )
        );

        if (auto_start) {
            is_recording_ = true;
            RCLCPP_INFO(
                this->get_logger(),
                "Auto-started recording to: %s",
                output_dir_.c_str()
            );
        }

        RCLCPP_INFO(this->get_logger(), "Image recorder node initialized");
        RCLCPP_INFO(
            this->get_logger(),
            "Output directory: %s",
            output_dir_.c_str()
        );
        RCLCPP_INFO(
            this->get_logger(),
            "Image format: %s",
            image_format_.c_str()
        );
        RCLCPP_INFO(
            this->get_logger(),
            "Recording status: %s",
            is_recording_ ? "ON" : "OFF"
        );
    }

private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        if (!is_recording_) {
            return;
        }

        try {
            // 将ROS图像消息转换为OpenCV格式
            cv_bridge::CvImagePtr cv_ptr;
            if (msg->encoding == "bgr8") {
                cv_ptr = cv_bridge::toCvCopy(
                    msg,
                    sensor_msgs::image_encodings::BGR8
                );
            } else if (msg->encoding == "rgb8") {
                cv_ptr = cv_bridge::toCvCopy(
                    msg,
                    sensor_msgs::image_encodings::RGB8
                );
            } else {
                cv_ptr = cv_bridge::toCvCopy(
                    msg,
                    sensor_msgs::image_encodings::BGR8
                );
            }

            // 生成文件名（包含时间戳）
            auto now = std::chrono::system_clock::now();
            auto time_t = std::chrono::system_clock::to_time_t(now);
            auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                          now.time_since_epoch()
                      )
                % 1000;

            std::stringstream ss;
            ss << std::put_time(std::localtime(&time_t), "%Y%m%d_%H%M%S");
            ss << "_" << std::setfill('0') << std::setw(3) << ms.count();
            ss << "_" << std::setfill('0') << std::setw(6) << image_count_;

            std::string filename =
                output_dir_ + "/image_" + ss.str() + "." + image_format_;

            // 保存图像
            if (cv::imwrite(filename, cv_ptr->image)) {
                image_count_++;
                if (image_count_ % 10 == 0) { // 每10张图像打印一次日志
                    RCLCPP_INFO(
                        this->get_logger(),
                        "Recorded %d images, latest: %s",
                        image_count_,
                        filename.c_str()
                    );
                }
            } else {
                RCLCPP_ERROR(
                    this->get_logger(),
                    "Failed to save image: %s",
                    filename.c_str()
                );
            }
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(
                this->get_logger(),
                "cv_bridge exception: %s",
                e.what()
            );
        } catch (const std::exception& e) {
            RCLCPP_ERROR(
                this->get_logger(),
                "Exception while saving image: %s",
                e.what()
            );
        }
    }

    void startRecordingCallback(
        const std::shared_ptr<std_srvs::srv::Empty::Request> /*request*/,
        std::shared_ptr<std_srvs::srv::Empty::Response> /*response*/
    ) {
        if (!is_recording_) {
            is_recording_ = true;
            image_count_ = 0;        // 重置计数器
            createOutputDirectory(); // 创建新的输出目录
            RCLCPP_INFO(
                this->get_logger(),
                "Started recording to: %s",
                output_dir_.c_str()
            );
        } else {
            RCLCPP_WARN(this->get_logger(), "Recording is already active");
        }
    }

    void stopRecordingCallback(
        const std::shared_ptr<std_srvs::srv::Empty::Request> /*request*/,
        std::shared_ptr<std_srvs::srv::Empty::Response> /*response*/
    ) {
        if (is_recording_) {
            is_recording_ = false;
            RCLCPP_INFO(
                this->get_logger(),
                "Stopped recording. Total images recorded: %d",
                image_count_
            );
        } else {
            RCLCPP_WARN(this->get_logger(), "Recording is already stopped");
        }
    }

    void createOutputDirectory() {
        // 如果不是绝对路径，则在当前工作目录下创建
        std::string base_dir = output_dir_;
        if (!std::filesystem::path(base_dir).is_absolute()) {
            auto now = std::chrono::system_clock::now();
            auto time_t = std::chrono::system_clock::to_time_t(now);

            std::stringstream ss;
            ss << std::put_time(std::localtime(&time_t), "%Y%m%d_%H%M%S");

            output_dir_ = base_dir + "_" + ss.str();
        }

        try {
            std::filesystem::create_directories(output_dir_);
            RCLCPP_INFO(
                this->get_logger(),
                "Created output directory: %s",
                output_dir_.c_str()
            );
        } catch (const std::filesystem::filesystem_error& e) {
            RCLCPP_ERROR(
                this->get_logger(),
                "Failed to create output directory: %s",
                e.what()
            );
        }
    }

    // 成员变量
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr
        image_subscription_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr start_recording_service_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr stop_recording_service_;

    std::string output_dir_;
    std::string image_format_;
    int image_count_;
    bool is_recording_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<ImageRecorderNode>();

    RCLCPP_INFO(node->get_logger(), "Image recorder node started");

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
