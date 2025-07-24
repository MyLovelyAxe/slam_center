import zmq
import rclpy
import struct
import pickle
import numpy as np

from std_msgs.msg import Header
from typing import Tuple, Dict
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from visualization_msgs.msg import Marker, MarkerArray
from scipy.spatial.transform import Rotation as R


class PointCloudCameraPoseTransfer(Node):


    def __init__(self):

        super().__init__('point_cloud_camera_pose_transfer')
        self.get_logger().info("Node initialized!")
        self.reconst_context = zmq.Context()
        self.reconst_socket = self.reconst_context.socket(zmq.SUB)
        self.reconst_socket.connect("tcp://127.0.0.1:5555")
        self.reconst_socket.setsockopt_string(zmq.SUBSCRIBE, "") # Subscribe to all messages

        self.pcd_publisher_ = self.create_publisher(PointCloud2, '/scene_reconst/point_cloud', 10)
        self.camera_publisher_ = self.create_publisher(MarkerArray, '/scene_reconst/camera_poses', 10)
        self.timer = self.create_timer(0.1, self.transfer_pcd_cam)
        self.get_logger().info('Waiting for point cloud and camera poses from zmq socket...')


    def transfer_pcd_cam(self):
        """ Receive point cloud and camera pose from zmq socket and publish them as PointCloud2 messages. """

        ### get point cloud and camera pose from zmq socket
        zmq_msg = self.reconst_socket.recv()
        self.get_logger().info(f'Received zmq message of size {len(zmq_msg)} bytes')
        point_cloud_positions, point_cloud_colors, cameras = self.extract_result(
            zmq_msg=zmq_msg,
            # downsample_size=10,
        )
        ### publish point cloud
        pcd_msg = self.create_point_cloud(
            xyz=point_cloud_positions,
            rgb=point_cloud_colors,
        )
        self.pcd_publisher_.publish(pcd_msg)
        ### publish markers for camera poses
        all_cameras_msg = MarkerArray()
        for cam_id, cam in cameras.items():
            cam_marker = self.create_camera_marker(
                marker_id=cam_id, 
                position=cam['translation'][0], # (1, 3) -> (3,)
                orientation=cam['rotation'], # (3, 3)
            )
            all_cameras_msg.markers.append(cam_marker)
        self.camera_publisher_.publish(all_cameras_msg)


    def extract_result(
        self,
        zmq_msg, #: bytes,
        downsample_size: int = 1,
        verbose: bool = False,
    ) -> Tuple[np.ndarray, np.ndarray, Dict[int, Dict[str, np.ndarray]]]:
        """ Extract point cloud and camera poses from the received zmq message.

        The message is an numpy.array for 3D reconstruction result, which consists of:

        - number of each partition
            shape: (3), i.e. [Num_Points, Num_Colors, Num_Cameras]
        - point cloud position
            shape: (N, 3), i.e. (Num_Points, XYZ)
        - point cloud colors
            shape: (N, 3), i.e. (Num_Colors, RGB)
        - camera poses
            shape: (M*4, 3), i.e. the extrinsics of M cameras concatenated vertically
            each extrinsics E is a 4x3 matrix:
                - E[0, :]: translation vector (3,), i.e. (X, Y, Z)
                - E[1:4, :]: rotation matrix (3, 3)

        :param zmq_msg: the received zmq message, which is a pickled numpy array
        :param verbose: if True, print out the received message

        :output point_cloud_positions: (N, 3) numpy array of point cloud positions
        :output point_cloud_colors: (N, 3) numpy array of point cloud colors
        :output cameras: dict of camera poses, each camera pose has translation (1x3) and rotation (3x3)
        """
        ### deserialize the zmq message
        array = pickle.loads(zmq_msg)
        ### numbers of each partition
        num_points, num_colors, num_cameras = array[0].astype(int)
        ### point cloud position
        point_cloud_positions = array[1:num_points + 1]
        ### point cloud colors
        # TODO: check if colors range is [0, 255] or [0, 1]
        point_cloud_colors = array[num_points + 1:num_points + num_colors + 1]
        ### camera poses
        camera_poses = array[num_points + num_colors + 1:]
        assert len(camera_poses) == num_cameras * 4, f"Wrong camera poses, i.e. len(camera_poses)={len(camera_poses)}, num_camerasx4={num_cameras}x4={num_cameras * 4}"
        cameras = dict()
        for i in range(num_cameras):
            camera_pose = camera_poses[i * 4:(i + 1) * 4]
            translation = camera_pose[0].reshape(1,3)
            rotation = camera_pose[1:4] # 3x3 rotation matrix
            cameras[i] = dict(
                translation=translation,
                rotation=rotation,
            )
        self.get_logger().info(f"Received 3D reconstruction message: num_points={num_points}, num_colors={num_colors}, num_cameras={num_cameras}")
        if verbose:
            self.get_logger().info(f"Received 3D reconstruction message: num_points={num_points}, num_colors={num_colors}, num_cameras={num_cameras}")
            self.get_logger().info(f"Point cloud positions shape: {point_cloud_positions.shape}")
            self.get_logger().info(f"Point cloud colors shape: {point_cloud_colors.shape}")
            for i, cam in cameras.items():
                self.get_logger().info("Camera {}: Translation={}, Rotation={}".format(i, cam['translation'], cam['rotation']))
        
        if not downsample_size == 1:
            self.get_logger().info(f"Downsampling point cloud by {downsample_size}...")
            point_cloud_positions = point_cloud_positions[::downsample_size]
            point_cloud_colors = point_cloud_colors[::downsample_size]

        return point_cloud_positions, point_cloud_colors, cameras


    def create_point_cloud(
        self, 
        xyz: np.ndarray, 
        rgb: np.ndarray,
    ) -> PointCloud2:
        """ Create a PointCloud2 message from XYZ and RGB data.

        :param xyz: position of point cloud, shape (N, 3), i.e. (Num_Points, XYZ)
        :param rgb: color of point cloud, shape (N, 3), i.e. (Num_Points, RGB) uint8 (0-255)

        :output point_cloud_msg: sensor_msgs/PointCloud2, for visualization in RViz
        """
        assert xyz.shape == rgb.shape and xyz.shape[1] == 3

        N = xyz.shape[0]

        fields = [
            PointField(name='x', offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8,  datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1)
        ]

        # Pack data
        cloud_data = []
        for i in range(N):
            x, y, z = xyz[i]
            r, g, b = rgb[i]
            rgb_uint32 = struct.unpack('I', struct.pack('BBBB', int(b), int(g), int(r), 0))[0]
            cloud_data.append(struct.pack('fffI', x, y, z, rgb_uint32))
        cloud_data = b''.join(cloud_data)

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'map'  # Set appropriately

        point_cloud_msg = PointCloud2()
        point_cloud_msg.header = header
        point_cloud_msg.height = 1
        point_cloud_msg.width = N
        point_cloud_msg.fields = fields
        point_cloud_msg.is_bigendian = False
        point_cloud_msg.point_step = 16  # 4 * float32 = 12, 1 * uint32 = 4
        point_cloud_msg.row_step = point_cloud_msg.point_step * N
        point_cloud_msg.is_dense = True
        point_cloud_msg.data = cloud_data

        return point_cloud_msg


    def create_camera_marker(
        self,
        marker_id: int, 
        position: np.ndarray,
        orientation: np.ndarray,
        frame_id: str = "map",
    ) -> Marker:
        """ Create a Marker message for visualizing camera pose in RViz. 
        
        :param marker_id: unique ID for the marker
        :param position: camera translation vector, shape (3,), i.e. (X, Y, Z)
        :param orientation: camera orientation, shape (3, 3)
        :param frame_id: the frame in which the camera pose is defined
        
        :output marker: visualization_msgs/Marker, for visualization in RViz
        """
        qx, qy, qz, qw = self.rotation_matrix_to_quaternion(rot_matrix=orientation)
        
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "camera"
        marker.id = marker_id
        marker.type = Marker.CUBE  # or .MESH_RESOURCE
        marker.action = Marker.ADD

        # Pose
        marker.pose.position.x = float(position[0])
        marker.pose.position.y = float(position[1])
        marker.pose.position.z = float(position[2])
        marker.pose.orientation.x = qx
        marker.pose.orientation.y = qy
        marker.pose.orientation.z = qz
        marker.pose.orientation.w = qw

        # Size and Color
        marker.scale.x = 0.1  # Width
        marker.scale.y = 0.05  # Height
        marker.scale.z = 0.02  # Depth
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.color.a = 1.0

        marker.lifetime.sec = 0  # 0 = forever

        return marker


    def rotation_matrix_to_quaternion(
        self,
        rot_matrix: np.ndarray,
    ) -> Tuple[float, float, float, float]:
        """ Convert a rotation matrix to a quaternion. 
        
        :param rot_matrix: (3, 3) numpy array representing the rotation matrix

        :output: (qx, qy, qz, qw) quaternion components
        """
        rotation = R.from_matrix(rot_matrix)
        qx, qy, qz, qw = rotation.as_quat()  # Returns (x, y, z, w)
        return qx, qy, qz, qw


def main(args=None):
    
    rclpy.init(args=args)
    node = PointCloudCameraPoseTransfer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
