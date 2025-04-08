
import rclpy
from rclpy.node import Node
from ethercat_utils_msgs.msg import GPIOArray
from std_msgs.msg import String


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(GPIOArray, '/digital_io_controller/commands', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.flag = False

    def timer_callback(self):
        msg = GPIOArray()
        msg.name = ['gpio/do0','gpio/do1']
        if self.flag:
            msg.data = [0.0,1.0]
        else:
            msg.data = [1.0,0.0]
        self.flag=not self.flag
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
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
