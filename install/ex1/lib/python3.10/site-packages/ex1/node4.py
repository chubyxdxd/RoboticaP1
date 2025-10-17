import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('node_4')
        self.subscription1 = self.create_subscription(
            Float64,
            'sensor_1',
            self.listener_callback1,
            10)
        self.subscription1  # prevent unused variable warning
        
        self.subscription2 = self.create_subscription(
            Float64,
            'sensor_2',
            self.listener_callback2,
            10)
        self.subscription2

        self.subscription3 = self.create_subscription(
            Float64,
            'sensor_3',
            self.listener_callback3,
            10)
        self.subscription3

        self.publisher_ = self.create_publisher(Float64, 'filtered_sensor', 10)
        timer_period = 5  # secondsself.get_logger().info('Publishing: "%s"' % msg.data)
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.val1 = 0
        self.val2 = 0
        self.val3 = 0

    def listener_callback1(self, msg):
        #self.get_logger().info('I heard: "%s"' % msg.data)
        self.val1 = msg.data

    def listener_callback2(self, msg):
        #self.get_logger().info('I heard: "%s"' % msg.data)
        self.val2 = msg.data
    def listener_callback3(self, msg):
        #self.get_logger().info('I heard: "%s"' % msg.data)
        self.val3 = msg.data
    def timer_callback(self):
        msg = Float64()
        msg.data = (self.val1 + self.val2 + self.val3)/3
        self.publisher_.publish(msg)
        #self.get_logger().info('Publishing: "%s"' % msg.data)
        self.get_logger().info(f'Publicando val1 = {self.val1}, val2 = {self.val2}, val3 = {self.val3} y al final mean = {msg.data}')

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()