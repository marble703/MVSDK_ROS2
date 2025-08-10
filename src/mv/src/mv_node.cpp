#include "mv/camera.hpp"

#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    // 创建节点
    auto node = std::make_shared<rclcpp::Node>("mv");

    auto logger = node->get_logger();

    RCLCPP_INFO(logger, "mv node initialized.");

    std::string camera_config_path = "/home/chen/MV_example/config/camera.config";

    Camera camera(camera_config_path);
    camera.init();

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher =
        node->create_publisher<sensor_msgs::msg::Image>(
            "mv/raw_image",
            rclcpp::SystemDefaultsQoS()
        );
    sensor_msgs::msg::Image image_msg;

    while (rclcpp::ok()) {
        auto start_time = std::chrono::steady_clock::now();

        // 获取图像
        cv::Mat frame = camera.get_frame();
        // cv::waitKey(10);

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

        auto end_time = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
            end_time - start_time
        );

        double fps = 1000.0 / duration.count();

        RCLCPP_INFO(logger, "Time: {%ld}ms, FPS: {%lf}", duration.count(), fps);
    }
    return 0;
}