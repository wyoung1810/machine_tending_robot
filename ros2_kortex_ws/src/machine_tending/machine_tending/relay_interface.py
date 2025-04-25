#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sainsmart_relay.relay import FTDIBitbangRelay

# Initialize the relay controller with the device ID
relay_controller = FTDIBitbangRelay(device_id='AB0NXT9F')

class RelayInterfaceNode(Node):

    def __init__(self):
        super().__init__('relay_interface')
        self.subscription = self.create_subscription(
            String,
            'arm_to_relay',
            self.sub_callback,
            10)
        self.subscription  # prevent unused variable warning

    def sub_callback(self, msg):
        if msg.data == "load":
            relay_controller.set_relay(1, 'on')
            relay_controller.set_relay(2, 'off')
            self.get_logger().info('"%s"' % msg.data)
        if msg.data == "unload":
            relay_controller.set_relay(1, 'off')
            relay_controller.set_relay(2, 'on')
            self.get_logger().info('"%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)

    relay_interface = RelayInterfaceNode()

    rclpy.spin(relay_interface)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    relay_interface.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()