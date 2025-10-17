import rclpy
from rclpy.node import Node
import random
from std_msgs.msg import Float64


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('node_2')
        self.publisher_ = self.create_publisher(Float64, 'sensor_2', 10)
        timer_period = 5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = Float64()
        msg.data = random.uniform(0.0,10.0)
        self.publisher_.publish(msg)
        #self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()