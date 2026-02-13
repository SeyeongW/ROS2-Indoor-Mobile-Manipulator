import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
# [추가] MoveIt 설정을 불러오기 위한 모듈
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description() -> LaunchDescription:
    # 1. Launch Arguments
    use_sim_time = LaunchConfiguration("use_sim_time")
    image_topic = LaunchConfiguration("image_topic")
    camera_info_topic = LaunchConfiguration("camera_info_topic")
    dictionary = LaunchConfiguration("dictionary")
    marker_size = LaunchConfiguration("marker_size")
    annotated_image_topic = LaunchConfiguration("annotated_image_topic")
    enable_pose = LaunchConfiguration("enable_pose")
    publish_debug = LaunchConfiguration("publish_debug")

    # 2. [핵심] OpenManipulator-X의 MoveIt 설정 로드
    # 이 설정이 없으면 C++ 노드가 robot_description을 찾지 못해 죽습니다.
    moveit_config = MoveItConfigsBuilder("open_manipulator_x", package_name="open_manipulator_x_moveit_config").to_moveit_configs()

    # 3. ArUco Node (Python)
    aruco_node = Node(
        package="omx_moveit_pick_cpp",
        executable="aruco_markers_node.py",
        name="aruco_markers_node",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "image_topic": image_topic,
                "camera_info_topic": camera_info_topic,
                "dictionary": dictionary,
                "marker_size": marker_size,
                "annotated_image_topic": annotated_image_topic,
                "enable_pose": enable_pose,          
                "publish_debug": publish_debug,      
            }
        ],
    )

    # 4. Picker Node (C++) - MoveIt 설정 주입
    pick_place_node = Node(
        package="omx_moveit_pick_cpp",
        executable="omx_moveit_pick_node",
        name="omx_moveit_pick_node",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            {"use_sim_time": use_sim_time}
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("image_topic", default_value="/camera/depth_camera/image_raw"),
            DeclareLaunchArgument("camera_info_topic", default_value="/camera/depth_camera/camera_info"),
            DeclareLaunchArgument("dictionary", default_value="DICT_5X5_100"),
            DeclareLaunchArgument("marker_size", default_value="0.06"),
            DeclareLaunchArgument("annotated_image_topic", default_value="/aruco/markers/image"),
            DeclareLaunchArgument("enable_pose", default_value="true"),     
            DeclareLaunchArgument("publish_debug", default_value="true"),   
            aruco_node,
            pick_place_node,
        ]
    )