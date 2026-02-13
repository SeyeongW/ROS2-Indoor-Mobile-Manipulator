// real_pick_node.cpp
// OpenManipulator-X: WAIT (no sweep) + pick closest visible ArUco marker
//  - Subscribes /aruco/markers to choose closest (min z) marker id
//  - Uses TF (link1 -> aruco_marker_<id>) and a 2-stage approach:
//      1) XY_ALIGN at fixed z_safe
//      2) DESCEND in steps to z_final
//  - Avoids target_z = mz + hover_offset (bad when camera is on end-effector)

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <control_msgs/action/gripper_command.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <ros2_aruco_interfaces/msg/aruco_markers.hpp>

#include <chrono>
#include <cmath>
#include <thread>
#include <vector>
#include <deque>
#include <algorithm>
#include <mutex>
#include <string>

using namespace std::chrono_literals;

using GripperCommand = control_msgs::action::GripperCommand;
using ArucoMarkers   = ros2_aruco_interfaces::msg::ArucoMarkers;

static double clamp(double v, double lo, double hi) {
  return std::max(lo, std::min(hi, v));
}
static bool isFinite(double v) {
  return std::isfinite(v);
}

static void operateGripper(rclcpp::Node::SharedPtr node,
                           rclcpp_action::Client<GripperCommand>::SharedPtr client,
                           double pos)
{
  if (!client->wait_for_action_server(2s)) {
    RCLCPP_ERROR(node->get_logger(), "Gripper action server not available");
    return;
  }
  auto goal = GripperCommand::Goal();
  goal.command.position = pos;
  goal.command.max_effort = 0.5;
  client->async_send_goal(goal);
  rclcpp::sleep_for(800ms);
}

enum class FSM {
  WAIT,          // idle at home/current pose until marker TF is fresh
  STABILIZE,     // require N consecutive fresh TFs, median filter
  APPROACH,      // 2-stage: XY_ALIGN then DESCEND
  GRASP,         // close gripper
  LIFT_AND_HOME  // lift and go home
};

enum class ApproachStage { XY_ALIGN, DESCEND };

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("real_pick_node");
  auto exec = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  exec->add_node(node);
  std::thread spinner([&exec](){ exec->spin(); });

  // ---------------- TF ----------------
  auto tf_buffer   = std::make_unique<tf2_ros::Buffer>(node->get_clock());
  auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

  // --------------- MoveIt -------------
  moveit::planning_interface::MoveGroupInterface arm(node, "arm");
  arm.setMaxVelocityScalingFactor(0.15);
  arm.setMaxAccelerationScalingFactor(0.15);
  arm.setPlanningTime(5.0);

  // position-only (4DOF)
  arm.setGoalPositionTolerance(0.03);
  arm.setGoalOrientationTolerance(3.14);

  // -------------- Gripper -------------
  auto gripper = rclcpp_action::create_client<GripperCommand>(
      node, "/gripper_controller/gripper_cmd");

  // ---------- USER SETTINGS ----------
  std::string base_frame    = "link1";          // robot base frame used for planning targets
  std::string markers_topic = "/aruco/markers"; // published by your aruco node

  // Gripper positions (may differ by setup)
  const double GRIP_OPEN  = 0.019;
  const double GRIP_CLOSE = -0.001;

  // TF freshness cutoff
  double max_tf_age_sec = 1.5;

  // Stabilization
  int stable_need = 5;

  // Failure handling
  int max_fail_before_wait = 3;

  // Filtering window (median)
  const size_t FILTER_N = 7;
  std::deque<double> fx, fy, fz;

  auto push_and_median = [&](std::deque<double>& dq, double v) -> double {
    dq.push_back(v);
    if (dq.size() > FILTER_N) dq.pop_front();
    std::vector<double> tmp(dq.begin(), dq.end());
    std::sort(tmp.begin(), tmp.end());
    return tmp[tmp.size()/2];
  };

  // ---- 2-stage approach parameters (tune here) ----
  double final_min_z  = 0.04;  // hard floor (avoid table/ground)
  double z_safe       = 0.25;  // stage1: XY alignment height (in base_frame)
  double z_final      = 0.12;  // stage2: final descend height before grasp
  double z_step_down  = 0.03;  // descend step
  double max_xy       = 0.25;  // clamp XY to avoid unreachable goals
  double max_z        = 0.35;  // clamp Z to avoid unreachable goals (conservative)

  // ---------- /aruco/markers latest cache ----------
  std::mutex mk_mtx;
  ArucoMarkers latest_markers;
  bool have_markers = false;

  auto markers_sub = node->create_subscription<ArucoMarkers>(
    markers_topic, 10,
    [&](const ArucoMarkers::SharedPtr msg){
      std::lock_guard<std::mutex> lk(mk_mtx);
      latest_markers = *msg;
      have_markers = true;
    }
  );

  // Choose closest marker by min pose.z in /aruco/markers (camera frame depth)
  auto selectClosestId = [&]() -> int {
    std::lock_guard<std::mutex> lk(mk_mtx);
    if (!have_markers) return -1;
    if (latest_markers.marker_ids.size() != latest_markers.poses.size()) return -1;
    if (latest_markers.marker_ids.empty()) return -1;

    int best_id = -1;
    double best_z = 1e9;

    for (size_t i=0; i<latest_markers.marker_ids.size(); i++) {
      const auto &p = latest_markers.poses[i].position;
      double z = p.z;
      if (!isFinite(z) || z <= 0.02 || z > 2.0) continue;
      if (z < best_z) {
        best_z = z;
        best_id = latest_markers.marker_ids[i];
      }
    }
    return best_id;
  };

  // Get fresh TF from base_frame to the marker frame
  auto getFreshMarkerTF = [&](const std::string& target_marker_frame,
                              double& ox, double& oy, double& oz) -> bool
  {
    try {
      if (!tf_buffer->canTransform(base_frame, target_marker_frame, tf2::TimePointZero, 50ms)) {
        return false;
      }
      auto t = tf_buffer->lookupTransform(base_frame, target_marker_frame, tf2::TimePointZero);

      rclcpp::Time now = node->get_clock()->now();
      rclcpp::Time stamp = t.header.stamp;
      double age = (now - stamp).seconds();

      if (age > max_tf_age_sec) {
        RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 1000,
                             "TF too old: %.2fs (ignore) frame=%s",
                             age, target_marker_frame.c_str());
        return false;
      }

      double x = t.transform.translation.x;
      double y = t.transform.translation.y;
      double z = t.transform.translation.z;

      if (!isFinite(x) || !isFinite(y) || !isFinite(z)) return false;

      // median filtering
      ox = push_and_median(fx, x);
      oy = push_and_median(fy, y);
      oz = push_and_median(fz, z);

      return true;
    } catch (...) {
      return false;
    }
  };

  // ---------- INIT ----------
  RCLCPP_INFO(node->get_logger(), ">> HOME");
  arm.setNamedTarget("home");
  arm.move();
  rclcpp::sleep_for(800ms);

  operateGripper(node, gripper, GRIP_OPEN);

  FSM state = FSM::WAIT;

  int stable_count = 0;
  int approach_fail = 0;

  // current target marker
  int target_id = -1;
  std::string target_marker_frame;

  // filtered marker pose in base_frame
  double mx=0, my=0, mz=0;

  // approach sub-stage
  ApproachStage approach_stage = ApproachStage::XY_ALIGN;
  double current_z_cmd = z_safe;

  rclcpp::Rate rate(10);

  while (rclcpp::ok()) {
    // 1) choose best marker id
    int best_id = selectClosestId();

    // 2) if target changes, reset filters/stability
    if (best_id != target_id) {
      target_id = best_id;
      stable_count = 0;
      fx.clear(); fy.clear(); fz.clear();

      if (target_id >= 0) {
        target_marker_frame = "aruco_marker_" + std::to_string(target_id);
        RCLCPP_INFO(node->get_logger(), ">> Target marker set to ID=%d (%s)",
                    target_id, target_marker_frame.c_str());
      } else {
        target_marker_frame.clear();
      }
    }

    // 3) get fresh TF for current target
    bool visible = false;
    if (!target_marker_frame.empty()) {
      visible = getFreshMarkerTF(target_marker_frame, mx, my, mz);
    }

    switch (state) {
      case FSM::WAIT: {
        stable_count = 0;
        fx.clear(); fy.clear(); fz.clear();

        if (visible) {
          RCLCPP_INFO(node->get_logger(), ">> Marker seen (ID=%d). Switching to STABILIZE", target_id);
          state = FSM::STABILIZE;
        } else {
          RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 2000,
                               ">> WAITING... (no fresh marker TF)");
        }
        break;
      }

      case FSM::STABILIZE: {
        if (!visible) {
          stable_count = 0;
          RCLCPP_WARN(node->get_logger(), ">> Lost marker TF. Back to WAIT");
          state = FSM::WAIT;
          break;
        }

        stable_count++;
        RCLCPP_INFO(node->get_logger(), ">> STABILIZE %d/%d (ID=%d x=%.3f y=%.3f z=%.3f)",
                    stable_count, stable_need, target_id, mx, my, mz);

        if (stable_count >= stable_need) {
          approach_fail = 0;
          approach_stage = ApproachStage::XY_ALIGN;
          current_z_cmd = z_safe;
          RCLCPP_INFO(node->get_logger(), ">> STABILIZE done. Switching to APPROACH (2-stage)");
          state = FSM::APPROACH;
        }
        break;
      }

      case FSM::APPROACH: {
        if (!visible) {
          RCLCPP_WARN(node->get_logger(), ">> Lost marker during approach. Back to STABILIZE");
          stable_count = 0;
          state = FSM::STABILIZE;
          break;
        }

        // Clamp XY (avoid impossible goals)
        double tx = clamp(mx, -max_xy, max_xy);
        double ty = clamp(my, -max_xy, max_xy);

        // Clamp Z bounds
        double z_safe_clamped  = clamp(z_safe,  final_min_z + 0.05, max_z);
        double z_final_clamped = clamp(z_final, final_min_z,        z_safe_clamped);

        if (approach_stage == ApproachStage::XY_ALIGN) {
          double tz = z_safe_clamped;

          RCLCPP_INFO(node->get_logger(),
                      ">> APPROACH[XY_ALIGN] target=(%.3f, %.3f, %.3f)", tx, ty, tz);

          arm.setStartStateToCurrentState();
          arm.setPositionTarget(tx, ty, tz);

          auto result = arm.move();

          if (result == moveit::core::MoveItErrorCode::SUCCESS) {
            approach_fail = 0;
            approach_stage = ApproachStage::DESCEND;
            current_z_cmd = z_safe_clamped;
            RCLCPP_INFO(node->get_logger(), ">> XY aligned. Switching to DESCEND");
          } else {
            approach_fail++;
            RCLCPP_ERROR(node->get_logger(), "XY_ALIGN move failed (%d/%d).",
                         approach_fail, max_fail_before_wait);

            if (approach_fail >= max_fail_before_wait) {
              RCLCPP_WARN(node->get_logger(), ">> Too many failures. Back to WAIT");
              state = FSM::WAIT;
            }
          }
          break;
        }

        if (approach_stage == ApproachStage::DESCEND) {
          // step down toward final height
          current_z_cmd -= z_step_down;
          if (current_z_cmd < z_final_clamped) current_z_cmd = z_final_clamped;

          double tz = current_z_cmd;

          RCLCPP_INFO(node->get_logger(),
                      ">> APPROACH[DESCEND] z=%.3f (final=%.3f) target=(%.3f, %.3f, %.3f)",
                      tz, z_final_clamped, tx, ty, tz);

          arm.setStartStateToCurrentState();
          arm.setPositionTarget(tx, ty, tz);

          auto result = arm.move();

          if (result == moveit::core::MoveItErrorCode::SUCCESS) {
            approach_fail = 0;
            if (std::abs(current_z_cmd - z_final_clamped) < 1e-3) {
              RCLCPP_INFO(node->get_logger(), ">> Reached final height. Switching to GRASP");
              state = FSM::GRASP;
            }
          } else {
            approach_fail++;
            RCLCPP_ERROR(node->get_logger(), "DESCEND move failed (%d/%d).",
                         approach_fail, max_fail_before_wait);

            if (approach_fail >= max_fail_before_wait) {
              RCLCPP_WARN(node->get_logger(), ">> Too many failures. Back to WAIT");
              state = FSM::WAIT;
            } else {
              // on failure, retreat a bit upward then retry
              current_z_cmd = std::min(z_safe_clamped, current_z_cmd + 0.05);
            }
          }
          break;
        }

        break;
      }

      case FSM::GRASP: {
        RCLCPP_INFO(node->get_logger(), ">> GRASPING! (ID=%d)", target_id);
        operateGripper(node, gripper, GRIP_CLOSE);
        state = FSM::LIFT_AND_HOME;
        break;
      }

      case FSM::LIFT_AND_HOME: {
        // Lift to safe height (do NOT use mz+offset; camera-on-EE makes that unstable)
        double lift_z = clamp(z_safe, final_min_z + 0.10, max_z);

        RCLCPP_INFO(node->get_logger(), ">> LIFT to z=%.3f then HOME", lift_z);

        arm.setStartStateToCurrentState();
        arm.setPositionTarget(clamp(mx, -max_xy, max_xy), clamp(my, -max_xy, max_xy), lift_z);
        arm.move();

        arm.setNamedTarget("home");
        arm.move();

        operateGripper(node, gripper, GRIP_OPEN);

        RCLCPP_INFO(node->get_logger(), ">> Done. Back to WAIT");
        state = FSM::WAIT;
        break;
      }
    }

    rate.sleep();
  }

  rclcpp::shutdown();
  spinner.join();
  return 0;
}