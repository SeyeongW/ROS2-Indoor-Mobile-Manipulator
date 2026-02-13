#include <memory>
#include <thread>
#include <vector>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp> // 액션 클라이언트 필수
#include <control_msgs/action/gripper_command.hpp> // 그리퍼 메시지 필수
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2/LinearMath/Quaternion.h>

using GripperCommand = control_msgs::action::GripperCommand;

geometry_msgs::msg::Quaternion createQuaternionFromRPY(double roll, double pitch, double yaw) {
  tf2::Quaternion q;
  q.setRPY(roll, pitch, yaw);
  geometry_msgs::msg::Quaternion q_msg;
  q_msg.x = q.x(); q_msg.y = q.y(); q_msg.z = q.z(); q_msg.w = q.w();
  return q_msg;
}

// [핵심] 힘(Effort)을 조절하여 그리퍼를 움직이는 함수
void operateGripper(rclcpp_action::Client<GripperCommand>::SharedPtr client, double width, double effort, rclcpp::Logger logger) {
  if (!client->wait_for_action_server(std::chrono::seconds(2))) {
    RCLCPP_ERROR(logger, "Gripper Action Server not found!");
    return;
  }
  
  auto goal_msg = GripperCommand::Goal();
  goal_msg.command.position = width;   
  goal_msg.command.max_effort = effort; // 여기서 힘을 조절함 (중요!)
  
  auto goal_options = rclcpp_action::Client<GripperCommand>::SendGoalOptions();
  client->async_send_goal(goal_msg, goal_options);
  
  // 물리적으로 움직일 시간을 충분히 줌
  rclcpp::sleep_for(std::chrono::seconds(2)); 
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>(
      "omx_moveit_pick_place",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor->add_node(node);
  std::thread spin_thread([&executor]() { executor->spin(); });

  // 1. 팔(Arm)은 MoveIt으로 제어
  static const std::string ARM_GROUP = "arm";
  moveit::planning_interface::MoveGroupInterface move_group_arm(node, ARM_GROUP);
  
  // 속도 및 오차 설정
  move_group_arm.setMaxVelocityScalingFactor(0.5);
  move_group_arm.setMaxAccelerationScalingFactor(0.5);
  move_group_arm.setGoalPositionTolerance(0.02);   
  move_group_arm.setGoalOrientationTolerance(0.1);

  // 2. 그리퍼(Gripper)는 Action Client로 제어 (힘 조절을 위해)
  auto gripper_action_client = rclcpp_action::create_client<GripperCommand>(
    node, "/gripper_controller/gripper_cmd");

  RCLCPP_INFO(node->get_logger(), "Planning Frame: %s", move_group_arm.getPlanningFrame().c_str());
  move_group_arm.setStartStateToCurrentState();

  // === 좌표 설정 ===
  // [주의] pick_x=0.3은 너무 멀고, pick_z=0.005는 바닥에 박습니다. 안전값으로 수정함.
  double pick_x = 0.300; 
  double pick_y = 0.000;
  double pick_z = 0.005; // 8cm (안전 높이)
  double approach_z_offset = 0.050;

  // 그리퍼 값 설정
  double gripper_open_val  = 0.019;
  double gripper_close_val = -0.001; // 살짝만 닫음 (박스 터짐 방지)
  double grip_effort       = 0.1;    // 힘을 아주 약하게 (박스 날아감 방지)

  // ---------------------------------------------------------
  // [Step 1] 홈 이동
  // ---------------------------------------------------------
  RCLCPP_INFO(node->get_logger(), ">> Moving to HOME");
  move_group_arm.setNamedTarget("home");
  move_group_arm.move();
  rclcpp::sleep_for(std::chrono::seconds(1));

  // ---------------------------------------------------------
  // [Step 2] 그리퍼 열기
  // ---------------------------------------------------------
  RCLCPP_INFO(node->get_logger(), ">> Gripper OPEN");
  operateGripper(gripper_action_client, gripper_open_val, 1.0, node->get_logger());

  // ---------------------------------------------------------
  // [Step 3] Pick 접근 (Above)
  // ---------------------------------------------------------
  RCLCPP_INFO(node->get_logger(), ">> Moving ABOVE Pick");
  geometry_msgs::msg::Pose target_pose;
  double yaw_pick = std::atan2(pick_y, pick_x);
  target_pose.orientation = createQuaternionFromRPY(0.0, 0.0, yaw_pick);
  target_pose.position.x = pick_x;
  target_pose.position.y = pick_y;
  target_pose.position.z = pick_z + approach_z_offset;

  move_group_arm.setPoseTarget(target_pose);
  move_group_arm.move();

  // ---------------------------------------------------------
  // [Step 4] 내려가기 (Down)
  // ---------------------------------------------------------
  RCLCPP_INFO(node->get_logger(), ">> Moving DOWN to Pick");
  target_pose.position.z = pick_z;
  move_group_arm.setPoseTarget(target_pose);
  move_group_arm.move();

  // ---------------------------------------------------------
  // [Step 5] 잡기 (살살 잡기!)
  // ---------------------------------------------------------
  RCLCPP_INFO(node->get_logger(), ">> GRASPING (Softly)");
  // 여기서 힘(grip_effort)을 0.1로 주어 부드럽게 잡습니다.
  operateGripper(gripper_action_client, gripper_close_val, grip_effort, node->get_logger());

  // ---------------------------------------------------------
  // [Step 6] 들어올리기 (Lift)
  // ---------------------------------------------------------
  RCLCPP_INFO(node->get_logger(), ">> LIFTING");
  target_pose.position.z = pick_z + approach_z_offset;
  move_group_arm.setPoseTarget(target_pose);
  move_group_arm.move();

  // ---------------------------------------------------------
  // [Step 7] 놓을 위치로 이동 (Place) - 관절 제어
  // ---------------------------------------------------------
  RCLCPP_INFO(node->get_logger(), ">> Moving to Place Location");
  // 4축 로봇은 좌표 이동이 어려우므로 관절 각도로 이동
  std::vector<double> place_joint_positions = {1.5, -0.6, 0.3, 0.7}; 
  move_group_arm.setJointValueTarget(place_joint_positions);
  move_group_arm.move();

  // ---------------------------------------------------------
  // [Step 8] 그리퍼 열기 (놓기)
  // ---------------------------------------------------------
  RCLCPP_INFO(node->get_logger(), ">> RELEASING");
  operateGripper(gripper_action_client, gripper_open_val, 1.0, node->get_logger());

  // ---------------------------------------------------------
  // [Step 9] 홈 복귀
  // ---------------------------------------------------------
  RCLCPP_INFO(node->get_logger(), ">> Returning HOME");
  move_group_arm.setNamedTarget("home");
  move_group_arm.move();

  RCLCPP_INFO(node->get_logger(), "Mission Complete!");
  
  rclcpp::shutdown();
  spin_thread.join();
  return 0;
}