#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <control_msgs/action/gripper_command.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/time.h>

#include <geometry_msgs/msg/transform_stamped.hpp>

#include <chrono>
#include <cmath>
#include <thread>
#include <future>
#include <vector>
#include <string>
#include <algorithm>

using namespace std::chrono_literals;
using GripperCommand = control_msgs::action::GripperCommand;

static double clamp(double v, double lo, double hi){
  return std::max(lo, std::min(hi, v));
}

static bool isFinite(double v){ return std::isfinite(v); }

static bool operateGripperBlocking(
  rclcpp::Node::SharedPtr node,
  rclcpp_action::Client<GripperCommand>::SharedPtr client,
  double pos, double effort,
  std::chrono::milliseconds server_wait = 1500ms,
  std::chrono::milliseconds result_wait = 4000ms
){
  if(!client->wait_for_action_server(server_wait)){
    RCLCPP_ERROR(node->get_logger(), "Gripper action server not available");
    return false;
  }

  GripperCommand::Goal goal;
  goal.command.position = pos;
  goal.command.max_effort = effort;

  auto fut_goal = client->async_send_goal(goal);
  if(fut_goal.wait_for(2s) != std::future_status::ready){
    RCLCPP_ERROR(node->get_logger(), "Gripper send_goal timeout");
    return false;
  }

  auto gh = fut_goal.get();
  if(!gh){
    RCLCPP_ERROR(node->get_logger(), "Gripper goal rejected");
    return false;
  }

  auto fut_res = client->async_get_result(gh);
  if(fut_res.wait_for(result_wait) != std::future_status::ready){
    RCLCPP_ERROR(node->get_logger(), "Gripper result timeout");
    return false;
  }

  return true;
}

enum class FSM { HOME, TRACK, GRASP, DONE };

static int find_index(const std::vector<std::string>& names, const std::string& key){
  for(size_t i=0;i<names.size();++i){
    if(names[i] == key) return static_cast<int>(i);
  }
  return -1;
}

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("omx_track_and_grasp");

  // Executor (TF/action 콜백 안정)
  auto exec = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  exec->add_node(node);
  std::thread spinner([&](){ exec->spin(); });

  // TF
  tf2_ros::Buffer tf_buffer(node->get_clock());
  tf2_ros::TransformListener tf_listener(tf_buffer);

  // MoveIt
  moveit::planning_interface::MoveGroupInterface arm(node, "arm");
  arm.setMaxVelocityScalingFactor(0.25);
  arm.setMaxAccelerationScalingFactor(0.25);
  arm.setPlanningTime(2.0);
  arm.setGoalPositionTolerance(0.02);
  arm.setGoalOrientationTolerance(3.14);

  // Frames / marker
  std::string camera_frame = node->declare_parameter<std::string>("camera_frame", "camera_color_optical_frame");
  std::string marker_frame = node->declare_parameter<std::string>("marker_frame", "aruco_marker_23");

  // Gripper
  auto gripper = rclcpp_action::create_client<GripperCommand>(node, "/gripper_controller/gripper_cmd");
  double GRIP_OPEN   = node->declare_parameter<double>("grip_open", 0.019);
  double GRIP_CLOSE  = node->declare_parameter<double>("grip_close", 0.0);
  double GRIP_EFFORT = node->declare_parameter<double>("grip_effort", 0.2);

  // Tracking gains / limits
  double K_yaw   = node->declare_parameter<double>("K_yaw", 0.6);
  double K_pitch = node->declare_parameter<double>("K_pitch", 0.6);
  double K_forward = node->declare_parameter<double>("K_forward", 0.4); // 접근용(관절3)

  double max_step_rad = node->declare_parameter<double>("max_step_rad", 0.08);

  // Center tolerance
  double yaw_tol_deg   = node->declare_parameter<double>("yaw_tol_deg", 4.0);
  double pitch_tol_deg = node->declare_parameter<double>("pitch_tol_deg", 4.0);
  double yaw_tol   = yaw_tol_deg   * M_PI / 180.0;
  double pitch_tol = pitch_tol_deg * M_PI / 180.0;

  // Distance gating for grasp
  double camera_to_grip = node->declare_parameter<double>("camera_to_grip", 0.08); // 카메라->그리퍼 끝 대략(미터)
  double grasp_distance = node->declare_parameter<double>("grasp_distance", 0.02); // 그리퍼 끝이 마커까지 2cm 남기고 닫기
  double grasp_tol      = node->declare_parameter<double>("grasp_tol", 0.02);      // ±2cm 허용
  int    grasp_need     = node->declare_parameter<int>("grasp_need", 5);           // N회 연속 만족 시 닫기

  // TF freshness
  double max_tf_age_sec = node->declare_parameter<double>("max_tf_age_sec", 0.7);

  // Command rate limit (preempt 방지)
  double cmd_hz = node->declare_parameter<double>("cmd_hz", 2.0); // 2Hz만 명령
  rclcpp::Duration cmd_period = rclcpp::Duration::from_seconds(1.0 / std::max(0.2, cmd_hz));
  rclcpp::Time last_cmd_time(0,0,RCL_ROS_TIME);

  // Joint mapping
  auto joint_names = arm.getJointNames();
  int idx_j1 = find_index(joint_names, "joint1");
  int idx_j2 = find_index(joint_names, "joint2");
  int idx_j3 = find_index(joint_names, "joint3");
  int idx_j4 = find_index(joint_names, "joint4");
  if(idx_j1 < 0 || idx_j2 < 0 || idx_j3 < 0){
    RCLCPP_ERROR(node->get_logger(), "Joint names not found in MoveIt group. Got names:");
    for(auto &n: joint_names) RCLCPP_ERROR(node->get_logger(), "  %s", n.c_str());
    rclcpp::shutdown();
    exec->cancel();
    spinner.join();
    return 1;
  }

  // Init
  FSM state = FSM::HOME;
  int grasp_ok_count = 0;

  rclcpp::Rate loop(20);

  while(rclcpp::ok()){
    if(state == FSM::HOME){
      RCLCPP_INFO(node->get_logger(), ">> HOME: go home, open gripper");
      arm.setNamedTarget("home");
      arm.move();
      rclcpp::sleep_for(300ms);
      (void)operateGripperBlocking(node, gripper, GRIP_OPEN, GRIP_EFFORT);
      grasp_ok_count = 0;
      state = FSM::TRACK;
    }

    if(state == FSM::TRACK){
      // TF: camera -> marker
      if(!tf_buffer.canTransform(camera_frame, marker_frame, tf2::TimePointZero, tf2::durationFromSec(0.1))){
        grasp_ok_count = 0;
        RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 1000,
                             "Waiting TF %s -> %s", camera_frame.c_str(), marker_frame.c_str());
        loop.sleep();
        continue;
      }

      auto tf = tf_buffer.lookupTransform(camera_frame, marker_frame, tf2::TimePointZero);

      // Freshness
      const rclcpp::Time now = node->get_clock()->now();
      const rclcpp::Time st(tf.header.stamp);
      const double age = (now - st).seconds();
      if(age > max_tf_age_sec){
        grasp_ok_count = 0;
        loop.sleep();
        continue;
      }

      const double x = tf.transform.translation.x;
      const double y = tf.transform.translation.y;
      const double z = tf.transform.translation.z;

      if(!isFinite(x)||!isFinite(y)||!isFinite(z) || z <= 0.05){
        grasp_ok_count = 0;
        loop.sleep();
        continue;
      }

      const double yaw_err   = std::atan2(x, z);
      const double pitch_err = std::atan2(-y, z);

      // “그리퍼 끝” 기준 거리 근사
      const double z_grip = z - camera_to_grip;
      const double dist_err = (z_grip - grasp_distance);

      const bool centered = (std::abs(yaw_err) < yaw_tol) && (std::abs(pitch_err) < pitch_tol);
      const bool dist_ok  = (std::abs(dist_err) < grasp_tol);

      RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 500,
                           "TRACK yaw=%.1fdeg pitch=%.1fdeg z=%.3f z_grip=%.3f dist_err=%.3f",
                           yaw_err*180.0/M_PI, pitch_err*180.0/M_PI, z, z_grip, dist_err);

      if(centered && dist_ok){
        grasp_ok_count++;
      }else{
        grasp_ok_count = 0;
      }

      if(grasp_ok_count >= grasp_need){
        RCLCPP_INFO(node->get_logger(), ">> GRASP condition satisfied (%d/%d).", grasp_ok_count, grasp_need);
        state = FSM::GRASP;
        continue;
      }

      // Rate-limit commands to avoid PREEMPTED
      if((now - last_cmd_time) < cmd_period){
        loop.sleep();
        continue;
      }
      last_cmd_time = now;

      // Joint servo (small increments)
      auto joints = arm.getCurrentJointValues();
      if(joints.size() != joint_names.size()){
        grasp_ok_count = 0;
        loop.sleep();
        continue;
      }

      // Centering increments (sign - : 너가 쓰던 기준)
      double d1 = clamp(-K_yaw   * yaw_err,   -max_step_rad, max_step_rad);
      double d2 = clamp(-K_pitch * pitch_err, -max_step_rad, max_step_rad);

      // Forward increment only when roughly centered (to keep marker in view)
      double d3 = 0.0;
      if(centered){
        d3 = clamp(+K_forward * dist_err, -max_step_rad, max_step_rad);
        // dist_err > 0 이면 멀다 -> 앞으로(팔을 뻗는 방향) 가야 함.
        // joint3의 부호가 반대면 여기만 -로 바꾸면 됨.
      }

      joints[idx_j1] += d1;
      joints[idx_j2] += d2;
      joints[idx_j3] += d3;

      // joint4는 유지 (카메라 방향 급변 방지)
      if(idx_j4 >= 0){
        joints[idx_j4] = joints[idx_j4];
      }

      arm.setStartStateToCurrentState();
      arm.setJointValueTarget(joints);
      auto res = arm.move();
      if(res != moveit::core::MoveItErrorCode::SUCCESS){
        // 실패해도 그냥 계속 추적 (HOME으로 튀지 않게)
        grasp_ok_count = 0;
      }
    }

    if(state == FSM::GRASP){
      RCLCPP_INFO(node->get_logger(), ">> Closing gripper");
      (void)operateGripperBlocking(node, gripper, GRIP_CLOSE, GRIP_EFFORT);
      state = FSM::DONE;
    }

    if(state == FSM::DONE){
      // 필요하면 lift/home 추가 가능. 일단 정지.
      RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 2000,
                           ">> DONE (holding).");
    }

    loop.sleep();
  }

  rclcpp::shutdown();
  exec->cancel();
  if(spinner.joinable()) spinner.join();
  return 0;
}