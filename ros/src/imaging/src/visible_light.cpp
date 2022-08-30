
#include "rclcpp/rclcpp.hpp"
#include "opencv2/opencv.hpp"

#include <cstdio>

int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;
  
  cv::VideoCapture camera(cv::CAP_V4L2);
  if (!camera.isOpened()) 
  {
        std::cerr << "ERROR: Could not open camera" << std::endl;
        return 1;
  }
  
  // Get the frame
  cv::Mat save_img;
  camera >> save_img;

  if(save_img.empty())
  {
    std::cerr << "Something is wrong with the webcam, could not get frame." << std::endl;
  }
// Save the frame into a file
  imwrite("test.jpg", save_img); // A JPG FILE IS BEING SAVED

  camera.release();
  return 0;
}
