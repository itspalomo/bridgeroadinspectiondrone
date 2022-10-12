// Image Processing libraries
#include <opencv2/opencv.hpp> // Processing and VL
#include "seek/seek.h" // Thermal Imaging

// ROS2 Specific Libraries
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"
#include "cv_bridge/cv_bridge.h"

// for Size
#include <opencv2/core/types.hpp>
// for CV_8UC3
#include <opencv2/core/hal/interface.h>
// for thermal image cap
#include <opencv2/highgui/highgui.hpp>
// for compressing the image
#include <image_transport/image_transport.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <chrono>
#include <memory>
#include <stdio.h>

using namespace std::chrono_literals;

class ImagePub : public rclcpp::Node {
  public:
    ImagePub() : Node("imagepub"), count_(0) {
        visible_pub =
            this->create_publisher<sensor_msgs::msg::Image>("imagepub/visible_light", 10);
            this->create_publisher<sensor_msgs::msg::Image>("imagepub/seek_thermal", 10);

        timer_ = this->create_wall_timer(
            100ms, std::bind(&ImagePub::timer_callback, this));
    }

  private:
    void timer_callback() {
        //Check if the cameras are working properly
        if (!seek.open()) 
            std::cerr << "ERROR: Failed to open seek thermal pro." << std::endl;
        if (!vl_cam.isOpened()) 
            std::cerr << "ERROR: Could not open visible light camera." << std::endl;

        cv::Mat visible_light_sensor;
        //get the frame and put it into a cv matrix
        vl_cam >> visible_light_sensor;
        if(vl_cam.empty())
            std::cerr << "Something is wrong with the webcam, could not get frame." << std::endl;


        sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", visible_light_sensor).toImageMsg();

        visible_pub->publish(*msg.get());
        std::cout << "Published visible light!" << std::endl;

        cv::Mat frame, grey_frame;       
        if (!seek.read(frame)) {
                std::cerr << "ERROR: No more LWIR img." << std::endl;
                return;
            }
    }

    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr visible_pub;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr thermal_pub;

    size_t count_;

    cv::VideoCapture vl_cam(10);
    LibSeek::SeekThermalPro seek();
};

int main(int argc, char *argv[]) {
    printf("Starting...");
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImagePub>());
    rclcpp::shutdown();
    return 0;
}