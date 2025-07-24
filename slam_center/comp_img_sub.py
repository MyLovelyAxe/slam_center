# ros2_zmq_pub.py
import rclpy
import zmq
import time
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage

"""
ros2 interface show sensor_msgs/msg/CompressedImage

# This message contains a compressed image.

std_msgs/Header header # Header timestamp should be acquisition time of image
	builtin_interfaces/Time stamp
		int32 sec
		uint32 nanosec
	string frame_id
                             # Header frame_id should be optical frame of camera
                             # origin of frame should be optical center of cameara
                             # +x should point to the right in the image
                             # +y should point down in the image
                             # +z should point into to plane of the image

string format                # Specifies the format of the data
                             #   Acceptable values:
                             #     jpeg, png, tiff

uint8[] data                 # Compressed image buffer
"""


def get_current_time_slot():
    return time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())

class CompressedImageSubscriberNode(Node):

    def __init__(self):
        super().__init__('compressed_image_subscriber_node')
        # Create a subscription to the CompressedImage topic
        self.compressed_img_sub = self.create_subscription(
            CompressedImage, '/camera/image_raw/compressed', self.image_callback, 10,
        )
        # Set up ZMQ context and PUB socket to send compressed images
        self.zmq_img_sender = zmq.Context()
        self.zmq_img_sender_socket = self.zmq_img_sender.socket(zmq.PUB)
        self.zmq_img_sender_socket.bind("tcp://127.0.0.1:5556") # Important: do not bind the same port with pcd_cam_vis node
        # Control the frequency of sending images
        self.latest_msg = None  # Store latest message
        self.timer = self.create_timer(0.1, self.send_latest_image)

    def image_callback(self, msg):
        """Store the latest compressed image message."""
        self.latest_msg = msg

    def send_latest_image(self):
        """Send the latest image at fixed frequency (1 Hz)."""

        if self.latest_msg is not None:
            msg = self.latest_msg
            self.get_logger().info(f"Sending: {msg.header.stamp.sec}.{msg.header.stamp.nanosec} compressed image in format of {msg.format}")
            self.zmq_img_sender_socket.send_multipart([
                msg.header.stamp.sec.to_bytes(8, byteorder='little', signed=False), # 8 bytes for int64
                msg.header.stamp.nanosec.to_bytes(4, byteorder='little', signed=False), # 4 bytes for uint32
                msg.format.encode(), # msg.format is string, convert it into bytes
                msg.data, # msg.data is bytes
            ])
            self.latest_msg = None  # Reset after sending


def main(args=None):
    rclpy.init(args=args)
    node = CompressedImageSubscriberNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
