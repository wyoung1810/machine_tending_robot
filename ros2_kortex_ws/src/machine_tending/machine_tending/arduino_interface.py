#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
# from pyfirmata2 import Arduino, util, INPUT
# import time

# Connect to the Arduino board
# board = Arduino("/dev/ttyACM0")  # use '/dev/ttyACM0' for Linux or 'COM4' for Windows

class ArduinoInterfaceNode(Node):

    arduino_state = "0"

    def __init__(self):
        super().__init__('arduino_interface')
        self.arduino_to_arm_pub = self.create_publisher(String, 'arduino_to_arm', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = String()
        # msg.data = read_arduino_state()
        # self.get_logger().info('Publishing: "%s"' % msg.data)

        msg.data = "3"
        if msg.data != self.arduino_state:
            self.arduino_state = msg.data
            self.arduino_to_arm_pub.publish(msg)
            self.get_logger().info('Publishing: "%s"' % msg.data)

# def read_arduino_state():
#     # Define the pins
#     digital_pins = [board.digital[2], board.digital[3]]  # D2, D3

#     # Set digital pins as inputs (assuming reading from them)
#     for d_pin in digital_pins:
#         d_pin.mode = INPUT
#         d_pin.enable_reporting()    

#     # Digital reads
#     digital_values = [pin.read() for pin in digital_pins]
#     return digital_values

def main(args=None):
    rclpy.init(args=args)

    arduino_interface = ArduinoInterfaceNode()

    rclpy.spin(arduino_interface)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    arduino_interface.destroy_node()
    rclpy.shutdown()
    board.exit()


if __name__ == '__main__':
    main()