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

#define SMOOTHING   10
#define COLORMAP    2
#define ROTATE      90

using namespace std::chrono_literals;

class ImagePub : public rclcpp::Node {
  public:
    ImagePub() : Node("imagepub"), count_(0) {
        //Creating the ros topics
        visible_pub =
            this->create_publisher<sensor_msgs::msg::Image>("imagepub/visible_light", 10);
        ir_pub = 
            this->create_publisher<sensor_msgs::msg::Image>("imagepub/seek_thermal", 10);

        //Open the cameras, check if the cameras are working properly
        //Visible light with index 0 and the v4l2 video backend
        if (!vl_cam->open(0, cv::CAP_V4L2)) 
            std::cerr << "ERROR: Could not open visible light camera." << std::endl;
        
        //IR, SeekThermal Compact Pro
        cam = &seekpro;
        if (!cam->open()) 
            std::cerr << "Something is wrong with the infrared, could not get frame." << std::endl;

        //Callback function that gets executes at 9Hz because of IR camera constraints.
        timer_ = this->create_wall_timer(
            111ms , std::bind(&ImagePub::timer_callback, this));
    }

  private:
    void timer_callback() {
        
        //Capture the frame
        VL_img = get_visible_light();
        IR_img = get_ir_light();

        //Publish new message to each topic
        sensor_msgs::msg::Image::SharedPtr msg_vl = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", VL_img).toImageMsg();
        sensor_msgs::msg::Image::SharedPtr msg_ir = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", IR_img).toImageMsg();

        visible_pub->publish(*msg_vl.get());
        ir_pub->publish(*msg_ir.get());
    }

    //Logic to grab a frame from the visible light image sensor
    cv::Mat get_visible_light()
    {
        cv::Mat visible_light_sensor;

        //get the frame and put it into a cv matrix
        *vl_cam >> visible_light_sensor;
        if(visible_light_sensor.empty())
            std::cerr << "Something is wrong with the webcam, could not get frame." << std::endl;
        
        return visible_light_sensor;
    }

    //Logic to grab frame from the IR sensor
    cv::Mat get_ir_light()
    {
        cv::Mat frame_u16, frame, avg_frame;

        // Aquire frames
        for (int i = 0; i < SMOOTHING; i++) {
            if (!cam->grab())
                std::cout << "no more LWIR img" << std::endl;

            cam->retrieve(frame_u16);
            frame_u16.convertTo(frame, CV_32FC1);

            if (avg_frame.rows == 0) 
                frame.copyTo(avg_frame);
            else
                avg_frame += frame;
        }


         // Average the collected frames
        avg_frame /= SMOOTHING;
        avg_frame.convertTo(frame_u16, CV_16UC1);

        cv::Mat frame_g8, outframe;

        //change type to apply colormap on IR
        cv::normalize(frame_u16, frame_u16, 0, 65535, cv::NORM_MINMAX);


        
        // Convert seek CV_16UC1 to CV_8UC1
        frame_u16.convertTo(frame_g8, CV_8UC1, 1.0 / 256.0);

        // Apply colormap: https://docs.opencv.org/master/d3/d50/group__imgproc__colormap.html#ga9a805d8262bcbe273f16be9ea2055a65
        if (COLORMAP != -1) {
            applyColorMap(frame_g8, outframe, COLORMAP);
        }
        else {
            cv::cvtColor(frame_g8, outframe, cv::COLOR_GRAY2BGR);
        }

        // Rotate image
        if (ROTATE == 90) {
            transpose(outframe, outframe);
            flip(outframe, outframe, 1);
        }
        else if (ROTATE == 180) {
            flip(outframe, outframe, -1);
        }
        else if (ROTATE == 270) {
            transpose(outframe, outframe);
            flip(outframe, outframe, 0);
        }

        return outframe;
    }

    void test_outfile(cv::Mat frame)
    {
        cv::imwrite("testout.jpg", frame);
        return;
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