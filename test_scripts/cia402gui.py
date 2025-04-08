import sys
import rclpy
from rclpy.node import Node
import threading
from PyQt5.QtWidgets import (
    QApplication, QWidget, QLabel, QVBoxLayout, QHBoxLayout, QLineEdit,
    QPushButton, QCheckBox, QGroupBox, QComboBox
)
from PyQt5.QtCore import Qt
from control_msgs.msg import InterfaceValue
from ethercat_utils_msgs.msg import GPIOArray

STATUS_BITS = {
    0: "Ready to switch on",
    1: "Switched on",
    2: "Operation enabled",
    3: "Fault",
    4: "Voltage enabled",
    5: "Quick stop  (negative logic)",
    6: "Switch on disabled",
    7: "Warning"
}

DEFAULT_CONTROL_BITS = {
    0: "Switch on",
    1: "Enable voltage",
    2: "Quick stop  (negative logic)",
    3: "Enable operation",
    4: "Operation mode specific",
    5: "Operation mode specific",
    6: "Halt",
    7: "Fault reset",
    8: "Manufacturer specific",
    9: "Manufacturer specific",
    10: "Manufacturer specific",
    11: "Manufacturer specific",
    12: "Manufacturer specific",
    13: "Manufacturer specific",
    14: "Manufacturer specific",
    15: "Manufacturer specific"
}

MOO_SPECIFIC_BITS = {
    1: {
        4: "New set point",
        5: "Change set immediately"
    },
    3: {
        4: "Reserved",
        5: "Reserved"
    },
    4: {
        4: "Torque setpoint enable",
        5: "Reserved"
    },
    6: {
        4: "Homing operation start",
        5: "Reserved"
    },
    7: {
        4: "Buffer enable",
        5: "Reserved"
    }
}

MODES_OF_OPERATION = {
    -1: "Manufacturer Specific",
     0: "No mode",
     1: "Profile Position Mode",
     2: "Velocity Mode",
     3: "Profile Velocity Mode",
     4: "Profile Torque Mode",
     6: "Homing Mode",
     7: "Interpolated Position Mode",
     8: "Cyclic Synchronous Position",
     9: "Cyclic Synchronous Velocity",
    10: "Cyclic Synchronous Torque"
}

STATE_MAP = [
    (0x004F, 0x0000, "Not Ready to Switch On"),
    (0x004F, 0x0040, "Switch On Disabled"),
    (0x006F, 0x0021, "Ready to Switch On"),
    (0x006F, 0x0023, "Switched On"),
    (0x006F, 0x0027, "Operation Enabled"),
    (0x006F, 0x0007, "Quick Stop Active"),
    (0x004F, 0x000F, "Fault Reaction Active"),
    (0x004F, 0x0008, "Fault")
]

class BrushlessInterfaceGUI(Node, QWidget):
    def __init__(self):
        Node.__init__(self, 'brushless_interface_gui')
        QWidget.__init__(self)
        self.setWindowTitle("Brushless CANopen 402 GUI")

        # Layouts
        main_layout = QVBoxLayout()
        self.setLayout(main_layout)

        # Status word and Moo display
        self.status_label = QLabel("Status Word: --")
        self.moo_label = QLabel("Mode of Operation Display: --")
        self.mode_name_label = QLabel("Mode: --")
        self.state_label = QLabel("402 State: --")
        main_layout.addWidget(self.status_label)
        main_layout.addWidget(self.state_label)
        main_layout.addWidget(self.moo_label)
        main_layout.addWidget(self.mode_name_label)

        # Status word bit checkboxes
        self.bit_checkboxes = {}
        bit_group = QGroupBox("Status Bits")
        bit_layout = QVBoxLayout()
        for bit, label in STATUS_BITS.items():
            cb = QCheckBox(f"{label} (bit {bit})")
            cb.setTristate(False)
            cb.setCheckable(True)
            cb.setFocusPolicy(Qt.NoFocus)
            cb.setEnabled(True)
            cb.setChecked(False)
            self.bit_checkboxes[bit] = cb
            bit_layout.addWidget(cb)
        bit_group.setLayout(bit_layout)
        main_layout.addWidget(bit_group)

        # Control word bit display and update
        self.ctrl_bits_checkboxes = {}
        self.ctrl_bit_labels = {}
        ctrl_bit_group = QGroupBox("Control Word Bits")
        ctrl_bit_layout = QVBoxLayout()
        for i in range(16):
            label = DEFAULT_CONTROL_BITS[i]
            cb = QCheckBox(f"{label} (bit {i})")
            cb.setTristate(False)
            cb.setCheckable(True)
            cb.setFocusPolicy(Qt.StrongFocus)
            cb.setEnabled(True)
            cb.setChecked(False)
            cb.stateChanged.connect(self.update_ctrl_input_from_bits)
            self.ctrl_bits_checkboxes[i] = cb
            self.ctrl_bit_labels[i] = label
            ctrl_bit_layout.addWidget(cb)
        ctrl_bit_group.setLayout(ctrl_bit_layout)
        main_layout.addWidget(ctrl_bit_group)

        # Control word input
        ctrl_layout = QHBoxLayout()
        self.ctrl_input = QLineEdit()
        self.ctrl_input.setPlaceholderText("0x00")
        self.ctrl_input.setReadOnly(True)
        send_btn = QPushButton("Send")
        send_btn.clicked.connect(self.send_control_word)
        ctrl_layout.addWidget(self.ctrl_input)
        ctrl_layout.addWidget(send_btn)
        main_layout.addLayout(ctrl_layout)

        # Mode of operation selection
        mode_layout = QHBoxLayout()
        self.mode_selector = QComboBox()
        for val, name in MODES_OF_OPERATION.items():
            self.mode_selector.addItem(f"{name} ({val})", val)
        mode_btn = QPushButton("Set Mode")
        mode_btn.clicked.connect(self.send_mode_of_operation)
        mode_layout.addWidget(self.mode_selector)
        mode_layout.addWidget(mode_btn)
        main_layout.addLayout(mode_layout)

        # ROS2 pub/sub
        self.subscription = self.create_subscription(
            InterfaceValue,
            '/brushless_controller/inputs',
            self.inputs_callback,
            10)

        self.publisher = self.create_publisher(
            GPIOArray,
            '/brushless_controller/commands',
            10)

        self.status_word = 0
        self.moo_display = -1

    def inputs_callback(self, msg):
        try:
            data = dict(zip(msg.interface_names, msg.values))
            self.status_word = int(data.get('joint_1/status_word', 0))
            self.moo_display = int(data.get('joint_1/moo_display', -1))
            self.update_gui()
        except Exception as e:
            self.get_logger().error(f"Error parsing input: {e}")

    def update_gui(self):
        self.status_label.setText(f"Status Word: 0x{self.status_word:04X}")
        self.moo_label.setText(f"Mode of Operation Display: {self.moo_display}")

        mode_name = MODES_OF_OPERATION.get(self.moo_display, "Unknown")
        self.mode_name_label.setText(f"Mode: {mode_name}")

        # Update operation-specific bits
        updated_labels = DEFAULT_CONTROL_BITS.copy()
        specific_bits = MOO_SPECIFIC_BITS.get(self.moo_display, {})
        for bit, label in specific_bits.items():
            updated_labels[bit] = label

        for bit, cb in self.ctrl_bits_checkboxes.items():
            cb.setText(f"{updated_labels[bit]} (bit {bit})")

        for bit, cb in self.bit_checkboxes.items():
            cb.setChecked(bool(self.status_word & (1 << bit)))

        state_str = "Unknown"
        for mask, value, label in STATE_MAP:
            if (self.status_word & mask) == value:
                state_str = label
                break

        self.state_label.setText(f"402 State: {state_str}")

    def update_ctrl_input_from_bits(self):
        value = 0
        for bit, cb in self.ctrl_bits_checkboxes.items():
            if cb.isChecked():
                value |= (1 << bit)
        self.ctrl_input.setText(f"0x{value:04X}")

    def send_control_word(self):
        text = self.ctrl_input.text().strip()
        try:
            value = int(text, 0)
            msg = GPIOArray()
            msg.name = ['joint_1/ctrl_word']
            msg.data = [float(value)]
            self.publisher.publish(msg)
            self.get_logger().info(f"Sent control word: 0x{value:04X}")
        except ValueError:
            self.get_logger().error("Invalid control word input")

    def send_mode_of_operation(self):
        mode_value = int(self.mode_selector.currentData())
        msg = GPIOArray()
        msg.name = ['joint_1/moo']
        msg.data = [float(mode_value)]
        self.publisher.publish(msg)
        self.get_logger().info(f"Sent mode of operation: {mode_value}")


def main():
    rclpy.init()
    app = QApplication(sys.argv)
    gui = BrushlessInterfaceGUI()
    gui.show()

    spin_thread = threading.Thread(target=rclpy.spin, args=(gui,), daemon=True)
    spin_thread.start()

    exit_code = app.exec_()
    gui.destroy_node()
    rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == '__main__':
    main()