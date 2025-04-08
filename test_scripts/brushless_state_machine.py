import rclpy
from rclpy.node import Node
from control_msgs.msg import InterfaceValue
from ethercat_utils_msgs.msg import GPIOArray

import threading
import time

STATUS_BITS = {
    0: "Ready to switch on",
    1: "Switched on",
    2: "Operation enabled",
    3: "Fault",
    4: "Voltage enabled",
    5: "Quick stop",
    6: "Switch on disabled",
    7: "Warning"
    # Add more if needed
}


class BrushlessInterfaceNode(Node):
    def __init__(self):
        super().__init__('brushless_interface_node')

        self.status_word = None
        self.last_status_word = None
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

        self.timer = self.create_timer(0.05, self.print_status)

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

    def print_status(self):
        if self.status_word is None:
            self.get_logger().info("Waiting for status word...")
            return
        if self.last_status_word is None:
            print_status_word = True
        elif self.last_status_word != self.status_word:
            print_status_word = True
        else:
            print_status_word = False
        
        self.last_status_word = self.status_word
        if print_status_word == False:
            return
            
        self.get_logger().info(f"\nStatus Word: {self.status_word}")
        for bit, desc in STATUS_BITS.items():
            bit_state = bool(self.status_word & (1 << bit))
            self.get_logger().info(f"{desc}: {bit_state}")

        self.get_logger().info(f"Mode of Operation Display (Moo Display): {self.moo_display}")
        
        
        self.get_logger().info(f"Press 'q' to exit")
        self.get_logger().info(f"Press '1' to Fault Reset")
        self.get_logger().info(f"Press '2' to Shutdown")
        self.get_logger().info(f"Press '3' to Switch On")
        self.get_logger().info(f"Press '4' to Enable Operation")
        self.get_logger().info(f"Press '5' to Disable Operation")  

    def keyboard_input_loop(self):
        actions = {
            '1': (0x80, "Fault Reset (0x80)"),
            '2': (0x06, "Shutdown (0x06)"),
            '3': (0x07, "Switch On (0x07)"),
            '4': (0x0F, "Enable Operation (0x0F)"),
            '5': (0x07, "Disable Operation (0x06)")
	    }

        print("\nControl Word Options:")
        for key, (value, desc) in actions.items():
            print(f"  Press {key} → {desc} (0x{value:02X})")
        print("  Press q to quit input loop")

        while True:
            try:
                key = input("\nEnter selection: ").strip()
                if key.lower() == 'q':
                    print("Exiting control input loop.")
                    break
                if key in actions:
                    value, desc = actions[key]
                    msg = GPIOArray()
                    msg.name = ['joint_1/ctrl_word']
                    msg.data = [float(value)]
                    self.publisher.publish(msg)
                    print(f"Published: {desc} (0x{value:02X})")
                else:
                    print("Invalid selection.")
            except Exception as e:
                print(f"Input error: {e}")



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

