#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import time

class ArduinoInterfaceNode(Node):

    def __init__(self):
        super().__init__('arduino_interface')
        self.arduino_state = "0,0"
        self.ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)  # use '/dev/ttyACM0' for Linux or 'COM4' for Windows and baud rate
        self.arduino_to_arm_pub = self.create_publisher(String, 'arduino_to_arm', 10)

        timer_period = 0.0001  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        if self.ser.in_waiting:
            msg = String()
            msg.data = self.ser.readline().decode().strip()
            self.get_logger().info('Reading: "%s"' % msg.data)

            if msg.data != self.arduino_state:
                try:
                    if msg.data == "0,0":
                        self.arduino_state = msg.data
                        self.get_logger().info('Neutral')
                    if msg.data == "1,0":
                        self.arduino_state = msg.data
                        self.get_logger().info('open')
                        msg.data = "load"
                        self.arduino_to_arm_pub.publish(msg)
                    if msg.data == "0,1":
                        self.arduino_state = msg.data
                        self.get_logger().info('close')
                        msg.data = "unload"
                        self.arduino_to_arm_pub.publish(msg)
                
                except ValueError:
                    self.get_logger().warn(f"Invalid data: {msg}")

def main(args=None):
    rclpy.init(args=args)

    arduino_interface = ArduinoInterfaceNode()

    rclpy.spin(arduino_interface)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    arduino_interface.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()