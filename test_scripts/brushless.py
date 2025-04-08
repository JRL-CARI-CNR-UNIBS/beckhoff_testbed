import rclpy
from rclpy.node import Node
from control_msgs.msg import InterfaceValue
from ethercat_utils_msgs.msg import GPIOArray

import threading
import time



class BrushlessInterfaceNode(Node):
    def __init__(self):
        super().__init__('brushless_interface_node')

        self.status_word = None
        self.moo_display = None

        self.subscription = self.create_subscription(
            InterfaceValue,
            '/brushless_controller/inputs',
            self.inputs_callback,
            10)

        self.publisher = self.create_publisher(
            GPIOArray,
            '/brushless_controller/commands',
            10)

        # Start thread for keyboard input
        self.keyboard_thread = threading.Thread(target=self.keyboard_input_loop)
        self.keyboard_thread.daemon = True
        self.keyboard_thread.start()

    def inputs_callback(self, msg):
        try:
            data = dict(zip(msg.interface_names, msg.values))
            self.status_word = int(data.get('joint_1/status_word', 0))
            self.moo_display = int(data.get('joint_1/moo_display', -1))
        except Exception as e:
            self.get_logger().error(f"Error parsing input message: {e}")

    def keyboard_input_loop(self):
        while True:
            try:
                val = input("\nEnter Position value (float): ")
                value = float(val.strip())

                msg = GPIOArray()
                msg.name = ['joint_1/position']
                msg.data = [value]

                self.publisher.publish(msg)
                print(f"Published position: {value}")
            except Exception as e:
                print(f"Invalid input: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = BrushlessInterfaceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

