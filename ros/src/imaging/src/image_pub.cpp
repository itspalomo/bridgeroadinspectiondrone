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
        ir_pub = 
            this->create_publisher<sensor_msgs::msg::Image>("imagepub/seek_thermal", 10);

        timer_ = this->create_wall_timer(
            100ms, std::bind(&ImagePub::timer_callback, this));
    }

  private:
    void timer_callback() {
         
        VL_img = get_visible_light(0);
        IR_img = get_ir_light();

        sensor_msgs::msg::Image::SharedPtr msg_vl = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", VL_img).toImageMsg();
        sensor_msgs::msg::Image::SharedPtr msg_ir = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", VL_img).toImageMsg();

        visible_pub->publish(*msg_vl.get());
        ir_pub->publish(*msg_ir.get());
    }

    cv::Mat get_visible_light(int cam_index)
    {
        //Check if the cameras are working properly
        if (!vl_cam->open(cam_index, cv::CAP_V4L2)) 
            std::cerr << "ERROR: Could not open visible light camera." << std::endl;

        cv::Mat visible_light_sensor;
        //get the frame and put it into a cv matrix
        *vl_cam >> visible_light_sensor;
        if(visible_light_sensor.empty())
            std::cerr << "Something is wrong with the webcam, could not get frame." << std::endl;
        
        return visible_light_sensor;
    }

    cv::Mat get_ir_light()
    {
        cv::Mat frame_u16, frame, avg_frame;
        std::string outfile = "/home/ubuntu/ros2_ws/bridgeroadinspectiondrone/image_processing/output.png";

        cam = &seekpro;

        if (!cam->open()) 
            std::cerr << "Something is wrong with the infrared, could not get frame." << std::endl;

        cam->retrieve(frame_u16);
        frame_u16.convertTo(frame, CV_16UC1);
        frame_u16 = frame;

        cv::Mat frame_g8, outframe;

        normalize(frame_u16, frame_u16, 0, 65535, cv::NORM_MINMAX);
        frame_u16.convertTo(frame_g8, CV_8UC1, 1.0 / 256.0);
        applyColorMap(frame_g8, outframe, 4);
        cv::imwrite(outfile, outframe);

        return outframe;
    }


    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr visible_pub;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr ir_pub;

    size_t count_;

    cv::Mat VL_img, IR_img;
    LibSeek::SeekThermalPro seekpro;
    LibSeek::SeekCam* cam;
        
    cv::VideoCapture *vl_cam = new cv::VideoCapture();
    
};


int main(int argc, char *argv[]) {

    printf("Starting...");
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImagePub>());
    rclcpp::shutdown(); 
    return 0;
}