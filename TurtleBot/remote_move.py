#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import socket
import threading
import time

class TurtleBotUDPController(Node):
    def __init__(self):
        super().__init__('udp_turtlebot_controller')
        self.publisher_ = self.create_publisher(Twist, '/tb4_08/cmd_vel', 10)

        #store latest command
        self.last_cmd_time = time.time()
        self.latest_linear_x = 0.0
        self.latest_angular_z = 0.0

        #ROS timer
        self.timer_ = self.create_timer(0.1, self.publish_cmd)  # 10 Hz
        #UDP listener
        self.udp_thread = threading.Thread(target=self.udp_listener, daemon=True)
        self.udp_thread.start()

    # receive commands from jetson
    def udp_listener(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind(('', 5025))

        while True:
            try:
                data, _ = sock.recvfrom(1024)
                msg = data.decode().strip()
                x_str, z_str = msg.split(',')
                self.latest_linear_x = float(x_str)
                self.latest_angular_z = float(z_str)
                self.last_cmd_time = time.time()
                self.get_logger().info(f'Received: x={self.latest_linear_x}, z={self.latest_angular_z}')
            except Exception as e:
                self.get_logger().warn(f'Invalid UDP message: {e}')

    def publish_cmd(self):
        twist = Twist()

        #if no message in 0.5 seconds, stop
        if time.time() - self.last_cmd_time > 0.5:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
        else:
            twist.linear.x = self.latest_linear_x
            twist.angular.z = self.latest_angular_z

        self.publisher_.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    controller = TurtleBotUDPController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
