# ros2_zmq_pub.py
import rclpy
from rclpy.node import Node
import zmq
import time

def get_current_time_slot():
    return time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())

class ZMQPublisherNode(Node):
    def __init__(self):
        super().__init__('zmq_publisher_node')

        # Set up ZMQ context and PUB socket
        self.zmq_context = zmq.Context() # since ROS2 node class has built-in self.context attribute, use self.zmq_context to avoid conflict
        self.zmq_socket = self.zmq_context.socket(zmq.PUB)
        self.zmq_socket.bind("tcp://127.0.0.1:5555")

        # Timer to send message every second
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        message = "hello from ROS2 at {}".format(get_current_time_slot())
        self.get_logger().info(f"Sending: {message}")
        self.zmq_socket.send_string(message)

def main(args=None):
    rclpy.init(args=args)
    node = ZMQPublisherNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
