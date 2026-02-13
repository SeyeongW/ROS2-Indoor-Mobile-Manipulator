#include <memory>
#include <thread>
#include <vector>
#include <cmath>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/gripper_command.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

using GripperCommand = control_msgs::action::GripperCommand;
using namespace std::chrono_literals;

// =========================================================================================
// [Helper] 쿼터니언 변환
// =========================================================================================
geometry_msgs::msg::Quaternion createQuaternionFromRPY(double roll, double pitch, double yaw) {
  tf2::Quaternion q;
  q.setRPY(roll, pitch, yaw);
  geometry_msgs::msg::Quaternion q_msg;
  q_msg.x = q.x(); q_msg.y = q.y(); q_msg.z = q.z(); q_msg.w = q.w();
  return q_msg;
}

// =========================================================================================
// [Helper] 그리퍼 제어 함수
// =========================================================================================
void operateGripper(rclcpp_action::Client<GripperCommand>::SharedPtr client, double width, double effort, rclcpp::Logger logger) {
  if (!client->wait_for_action_server(std::chrono::seconds(2))) {
    RCLCPP_ERROR(logger, "Gripper Action Server not found!");
    return;
  }
  auto goal_msg = GripperCommand::Goal();
  goal_msg.command.position = width;   
  goal_msg.command.max_effort = effort; 
  auto goal_options = rclcpp_action::Client<GripperCommand>::SendGoalOptions();
  client->async_send_goal(goal_msg, goal_options);
  rclcpp::sleep_for(std::chrono::seconds(1)); 
}

// =========================================================================================
// [Main Class] 로직을 깔끔하게 관리하기 위해 클래스 구조 사용 권장하지만, 
// 기존 스타일 유지를 위해 함수형으로 작성하되 기능을 모듈화함.
// =========================================================================================

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("omx_smart_tracker");
  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor->add_node(node);
  std::thread spin_thread([&executor]() { executor->spin(); });

  // TF & MoveIt Setup
  auto tf_buffer = std::make_unique<tf2_ros::Buffer>(node->get_clock());
  auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

  static const std::string ARM_GROUP = "arm";
  moveit::planning_interface::MoveGroupInterface move_group_arm(node, ARM_GROUP);
  
  // 추적 중에는 너무 빠르면 마커를 놓치므로 속도 조절
  move_group_arm.setMaxVelocityScalingFactor(0.5); 
  move_group_arm.setGoalPositionTolerance(0.01);   // 정밀하게
  move_group_arm.setGoalOrientationTolerance(0.1);

  auto gripper_action_client = rclcpp_action::create_client<GripperCommand>(
    node, "/gripper_controller/gripper_cmd");

  // 초기화
  move_group_arm.setStartStateToCurrentState();
  operateGripper(gripper_action_client, 0.019, 1.0, node->get_logger()); // 그리퍼 열기

  // =========================================================
  // 설정 변수
  // =========================================================
  std::string target_frame = "aruco_marker_0"; 
  std::string base_frame = "link1"; 

  // 탐색 웨이포인트 (바닥 -> 위로)
  std::vector<std::vector<double>> scan_waypoints = {
      {0.0, 0.20, 0.10, 1.50},   // 1. 발밑
      {0.0, 0.00, 0.20, 1.20},   // 2. 약간 앞
      {0.0, -0.50, 0.40, 1.00},  // 3. 중간
      {0.0, -0.90, 0.50, 0.80}   // 4. 멀리
  };

  // =========================================================
  // [Logic] 스마트 추적 루프
  // =========================================================
  RCLCPP_INFO(node->get_logger(), ">> [START] Smart Tracking Initiated...");

  // 현재 목표 거리 (처음에는 높은 곳 25cm에서 시작 -> 점점 3cm까지 줄임)
  double current_height_target = 0.25; 
  double final_grasp_height = 0.035; // 최종 잡기 높이 (3.5cm)
  double step_size = 0.05; // 한번에 5cm씩 접근

  // 상태 변수
  bool marker_visible = false;
  double marker_x = 0.0, marker_y = 0.0, marker_z = 0.0;
  int scan_idx = 0;

  while (rclcpp::ok()) {
      // 1. 마커 확인
      marker_visible = false;
      try {
          if (tf_buffer->canTransform(base_frame, target_frame, tf2::TimePointZero, 1s)) {
              auto t = tf_buffer->lookupTransform(base_frame, target_frame, tf2::TimePointZero);
              marker_x = t.transform.translation.x;
              marker_y = t.transform.translation.y;
              marker_z = t.transform.translation.z;
              marker_visible = true;
          }
      } catch (tf2::TransformException & ex) {}

      // 2. 마커를 놓쳤을 경우 -> "다시 찾아라!" (Re-Scan)
      if (!marker_visible) {
          RCLCPP_WARN(node->get_logger(), ">> [LOST] Marker lost! Scanning area %d...", scan_idx+1);
          
          // 웨이포인트로 이동해서 다시 찾기
          move_group_arm.setJointValueTarget(scan_waypoints[scan_idx]);
          move_group_arm.move();
          
          rclcpp::sleep_for(std::chrono::milliseconds(500)); // 카메라 안정화
          scan_idx = (scan_idx + 1) % scan_waypoints.size(); // 다음 스캔 위치 준비
          
          // 이번 루프는 건너뛰고 다시 마커 확인하러 감
          continue; 
      }

      // 3. 마커를 찾았을 경우 -> 거리 확인
      RCLCPP_INFO(node->get_logger(), ">> [TRACKING] Marker at (%.3f, %.3f), Height Offset: %.3f", 
                  marker_x, marker_y, current_height_target);

      // 이미 충분히 가까우면(목표 높이 도달) 루프 탈출 -> 잡기
      if (current_height_target <= final_grasp_height + 0.005) { 
          RCLCPP_INFO(node->get_logger(), ">> [READY] Reached target distance. Ready to Grasp.");
          break; 
      }

      // 4. 접근 이동 (Step Approach)
      // 현재 마커 좌표(X,Y)의 'current_height_target' 높이로 이동
      // 즉, 마커 위를 계속 맴돌면서 고도를 낮춤
      
      geometry_msgs::msg::Pose approach_pose;
      double yaw = std::atan2(marker_y, marker_x); 
      approach_pose.orientation = createQuaternionFromRPY(0.0, 0.0, yaw);
      
      // 마커 좌표는 TF로 계속 업데이트되므로 가장 최신 좌표 사용 (보정 효과)
      approach_pose.position.x = marker_x;
      approach_pose.position.y = marker_y;
      
      // 높이는 마커 바닥(marker_z) + 현재 목표 높이(current_height_target)
      // *주의: TF의 Z가 바닥 0이 아닐 수 있으므로 절대 높이 대신 상대 높이 고려
      // 가제보 환경에서 marker_z는 보통 마커 자체의 높이임.
      // 우리는 로봇 베이스 기준 Z로 제어하므로, 안전하게 고정값 바닥 높이(0.0) + 타겟 높이 사용
      // 혹은 marker_z가 믿을만하다면 marker_z + height 사용. 여기선 marker_z(마커표면) + height로 설정
      approach_pose.position.z = marker_z + current_height_target;

      // 너무 낮으면 바닥 충돌 방지
      if (approach_pose.position.z < final_grasp_height) approach_pose.position.z = final_grasp_height;

      move_group_arm.setPoseTarget(approach_pose);
      auto result = move_group_arm.move();

      if (result == moveit::core::MoveItErrorCode::SUCCESS) {
          // 이동 성공했으면 더 가까이 가기 위해 목표 높이 줄임
          current_height_target -= step_size; 
          if (current_height_target < final_grasp_height) current_height_target = final_grasp_height;
          
          // 스캔 인덱스 초기화 (잘 찾고 있으니까)
          scan_idx = 0; 
      } else {
          // 이동 실패했으면(IK 실패 등) 다시 스캔하거나, 조금 덜 이동하도록 로직 유지
          RCLCPP_WARN(node->get_logger(), ">> [FAIL] Move failed. Retrying...");
      }
      
      // 잠깐 대기 후 다시 루프 (다시 마커 위치 확인 -> 보정 -> 이동)
      rclcpp::sleep_for(std::chrono::milliseconds(500));
  }

  // =========================================================
  // [Final Grasp] 루프 탈출 후 잡기
  // =========================================================
  RCLCPP_INFO(node->get_logger(), ">> GRASPING SEQUENCE START");

  // 1. 마지막으로 미세 정렬하며 꽉 잡을 위치로 하강
  // 이때는 마커가 너무 가까워서 안 보일 수 있으므로, 마지막 기억된 좌표(marker_x, y)를 믿고 내려감
  geometry_msgs::msg::Pose final_pose;
  double yaw_final = std::atan2(marker_y, marker_x);
  final_pose.orientation = createQuaternionFromRPY(0.0, 0.0, yaw_final);
  final_pose.position.x = marker_x;
  final_pose.position.y = marker_y;
  final_pose.position.z = final_grasp_height; // 3.5cm

  move_group_arm.setPoseTarget(final_pose);
  move_group_arm.move();

  // 2. 잡기
  operateGripper(gripper_action_client, -0.001, 0.1, node->get_logger());
  rclcpp::sleep_for(std::chrono::seconds(1));

  // 3. 들기
  RCLCPP_INFO(node->get_logger(), ">> LIFTING");
  final_pose.position.z += 0.15;
  move_group_arm.setPoseTarget(final_pose);
  move_group_arm.move();

  // 4. 놓기 (관절 제어)
  RCLCPP_INFO(node->get_logger(), ">> PLACING");
  std::vector<double> place_joint_positions = {1.5, -0.6, 0.3, 0.7}; 
  move_group_arm.setJointValueTarget(place_joint_positions);
  move_group_arm.move();

  operateGripper(gripper_action_client, 0.019, 1.0, node->get_logger());
  move_group_arm.setNamedTarget("home");
  move_group_arm.move();

  RCLCPP_INFO(node->get_logger(), "Mission Complete!");
  
  rclcpp::shutdown();
  spin_thread.join();
  return 0;
}