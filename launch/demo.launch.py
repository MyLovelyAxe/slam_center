import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():


    rviz_config_file = os.path.join(
        get_package_share_directory("slam_center"),
        "config",
        "slam_center_rviz_config.rviz",
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    # receive compressed image from topic and sent to zmq socket
    send_comp_img_node = Node(
        package='slam_center',
        executable='send_comp_img',
        name='send_comp_img',
    )

    # receive point cloud and camera pose from zmq socket
    pcd_cam_vis_node = Node(
        package='slam_center',
        executable='visualize_pcd_cam',
        name='visualize_pcd_cam',
    )

    return LaunchDescription([
        rviz_node,
        send_comp_img_node,
        pcd_cam_vis_node,
    ])