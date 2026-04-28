from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("model_name", default_value="yolov8n.pt"),
        DeclareLaunchArgument("confidence", default_value="0.40"),
        DeclareLaunchArgument("device",     default_value="cpu"),

        Node(
            package="ros_gz_bridge", executable="parameter_bridge",
            name="gz_ros_bridge",
            arguments=[
                "/world/default/model/x500_depth_0/link/camera_link/sensor/IMX214/image"
                "@sensor_msgs/msg/Image@gz.msgs.Image",
                "/world/default/model/x500_depth_0/link/camera_link/sensor/IMX214/camera_info"
                "@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo",
                "/depth_camera@sensor_msgs/msg/Image@gz.msgs.Image",
                "/depth_camera/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked",
                "--ros-args",
                "-r", "/world/default/model/x500_depth_0/link/camera_link/sensor/IMX214/image:=/camera/color/image",
                "-r", "/world/default/model/x500_depth_0/link/camera_link/sensor/IMX214/camera_info:=/camera/color/camera_info",
                "-r", "/depth_camera:=/camera/depth/image",
                "-r", "/depth_camera/points:=/camera/depth/points",
            ],
            output="screen",
        ),

        Node(
            package="yolo_detector", executable="yolo_node", name="yolo_detector",
            output="screen",
            parameters=[{
                "model_name":           LaunchConfiguration("model_name"),
                "confidence_threshold": LaunchConfiguration("confidence"),
                "device":               LaunchConfiguration("device"),
            }],
        ),

        Node(
            package="depth_estimator", executable="depth_node", name="depth_estimator",
            output="screen",
            parameters=[{"max_depth_m": 10.0, "voxel_size": 0.05,
                         "ransac_iters": 80, "ransac_dist_thresh": 0.06}],
        ),
    ])
