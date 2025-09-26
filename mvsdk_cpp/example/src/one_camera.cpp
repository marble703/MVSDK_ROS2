#include "camera.hpp"

int main(){
    std::string camera_config_path = "config/camera.config";
    Camera camera(camera_config_path);
    cv::Mat frame;

    while (true){
        frame = camera.imread();
        cv::imshow("frame", frame);
        cv::waitKey(1);
    }
    return 0;
}