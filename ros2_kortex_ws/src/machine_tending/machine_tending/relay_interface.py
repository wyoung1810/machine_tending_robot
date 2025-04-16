#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sainsmart_relay.relay import FTDIBitbangRelay

# Initialize the relay controller with the device ID
relay_controller = FTDIBitbangRelay(device_id='AB0NXT9F')

class MinimalPublisher(Node):

    relay_state = "00000000"

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'relay_to_arm', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = read_relay_state()
        if msg.data != self.relay_state:
            self.relay_state = msg.data
            self.publisher_.publish(msg)
            self.get_logger().info('Publishing: "%s"' % msg.data)

def read_relay_state():
    # Retrieve the current relay state
    state = relay_controller.get_relay_state()

    # Convert relay state to a readable string
    return f"{state:08b}"

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