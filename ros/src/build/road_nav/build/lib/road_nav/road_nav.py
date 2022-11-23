import rclpy
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import os
from rclpy.node import Node
import std_msgs.msg
from sensor_msgs.msg import Image as Image_msg

class RoadNav(Node):

    def __init__(self):
        super().__init__('road_nav')
        #self.publisher_ = self.create_publisher(std_msgs.msg.String, 'road_nav/fog_line', 10)
        #self.publisher_ = self.create_publisher(std_msgs.msg.String, 'road_nav/analysis_mode', 10)
        self.vl_sub = self.create_subscription(
            Image_msg,
            'imagepub/visible_light',
            self.vl_callback,
            10)
        
        self.ir_sub = self.create_subscription(
            Image_msg,
            'imagepub/seek_thermal',
            self.ir_callback,
            10)
        
        self.vl_sub  # prevent unused variable warning
        self.ir_sub
        self.i = 0
        self.bridge = CvBridge()

    def vl_callback(self, msg):
        # Display the message on the console
        self.get_logger().info('Receiving visible light frame')

        try:
            #convert from ros image to opencv image
            visible_light_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)
        
        # Display image
        cv2.imwrite("/home/ubuntu/ros2_ws/bridgeroadinspectiondrone/ros/data_11_23/visible" + str(self.i) + ".jpg", visible_light_image)
        self.i += 1
    

    def ir_callback(self, msg):
        # Display the message on the console
        self.get_logger().info('Receiving infrared frame')

        try:
            #convert from ros image to opencv image
            ir_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)
        
        # Display image
        cv2.imwrite("/home/ubuntu/ros2_ws/bridgeroadinspectiondrone/ros/data_11_23/ir" + str(self.i) + ".jpg", ir_image)



def main(args=None):
    rclpy.init(args=args)

    fog_line = RoadNav()

    rclpy.spin(fog_line)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    fog_line.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()