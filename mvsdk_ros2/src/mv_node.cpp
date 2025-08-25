#include "camera.hpp"

#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/float32.hpp>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    // 创建节点
    auto node = std::make_shared<rclcpp::Node>("mv");

    auto logger = node->get_logger();

    RCLCPP_INFO(logger, "mv node initialized.");

    std::string camera_config_path = "config/camera.config";

    Camera camera(camera_config_path);

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher =
        node->create_publisher<sensor_msgs::msg::Image>(
            "mv/raw_image",
            rclcpp::SystemDefaultsQoS()
        );
    sensor_msgs::msg::Image image_msg;

    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr
        duration_time_publisher =
            node->create_publisher<std_msgs::msg::Float32>(
                "mv/duration_time",
                rclcpp::SystemDefaultsQoS()
            );
    std_msgs::msg::Float32 duration_msg;

    cv::Mat frame;
    std::shared_ptr<cv::Mat> frame_ptr = std::make_shared<cv::Mat>();

    while (rclcpp::ok()) {
        auto start_time = std::chrono::high_resolution_clock::now();

        // 获取图像
        camera.readFrame();
        frame = camera.getFrame();
        // cv::waitKey(1);

        // debug状态发布原始图像
        image_msg.header.stamp = node->now();
        image_msg.height = frame.rows;
        image_msg.width = frame.cols;
        image_msg.encoding = "bgr8";
        image_msg.is_bigendian = false;
        image_msg.step = frame.cols * frame.elemSize();
        size_t size = frame.cols * frame.rows * frame.elemSize();
        image_msg.data.resize(size);
        memcpy(image_msg.data.data(), frame.data, size);
        image_publisher->publish(image_msg);

        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(
            end_time - start_time
        );

        double fps = 1000000.0 / duration.count();

        // 实际使用可以改为 DEBUG
        RCLCPP_DEBUG(
            logger,
            "Time: {%ld}us, FPS: {%lf}",
            duration.count(),
            fps
        );
        duration_msg.data = duration.count();
        duration_time_publisher->publish(duration_msg);
    }
    return 0;
}