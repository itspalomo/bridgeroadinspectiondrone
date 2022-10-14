import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class RoadNav(Node):

    def __init__(self):
        super().__init__('road_nav')
        self.publisher_ = self.create_publisher(String, 'road_nav/fog_line', 10)
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)

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