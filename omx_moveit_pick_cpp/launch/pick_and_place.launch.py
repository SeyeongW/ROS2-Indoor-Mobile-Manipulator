from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # 1. OpenManipulator-X의 MoveIt 설정을 가져옵니다.
    # (이미 설치된 open_manipulator_x_moveit_config 패키지에서 읽어옴)
    moveit_config = MoveItConfigsBuilder("open_manipulator_x", package_name="open_manipulator_x_moveit_config").to_moveit_configs()

    # 2. 내 노드(omx_grip_node)를 실행할 때, 위에서 가져온 설정을 먹여줍니다.
    pick_place_node = Node(
        package="omx_moveit_pick_cpp",
        executable="omx_grip_node",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics, # [핵심] 이걸 넣어줘야 경고가 사라짐
        ],
    )

    return LaunchDescription([
        pick_place_node
    ])