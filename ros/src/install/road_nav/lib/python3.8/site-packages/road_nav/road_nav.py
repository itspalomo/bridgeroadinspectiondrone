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
        self.publisher_ = self.create_publisher(std_msgs.msg.String, 'road_nav/fog_line', 10)
        #self.publisher_ = self.create_publisher(std_msgs.msg.String, 'road_nav/analysis_mode', 10)
        self.subscription = self.create_subscription(
            Image_msg,
            'imagepub/visible_light',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.bridge = CvBridge()

    def timer_callback(self):
        msg = std_msgs.msg.String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1
    
    # sensor_msgs::msg::Image::SharedPtr msg = 
    # cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", visible_light_sensor).toImageMsg();

    def listener_callback(self, msg):
        try:
            #convert from ros image to opencv image
            visible_light_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)
            #HoughLine detection
        while True:
            ret,frame = visible_light_image.read()
            img = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            edge = cv2.Canny(img,300,350)
            lines =cv2.HoughLinesP(edge, rho = 1, theta = 1*np.pi/180, threshold = 100, minLineLength = 170, maxLineGap = 10)

            try:
                for i in lines:
                    x1,x2,y1,y2 = i[0]
                    cv2.line(frame,(x1,x2), (y1,y2), (0,255,0),3)
            except:
                print(f'No lines detected')


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