#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <control_msgs/action/gripper_command.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/exceptions.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <ros2_aruco_interfaces/msg/aruco_markers.hpp>

#include <chrono>
#include <thread>
#include <mutex>
#include <vector>
#include <algorithm>
#include <cmath>
#include <string>
#include <future>

using namespace std::chrono_literals;

using GripperCommand = control_msgs::action::GripperCommand;
using ArucoMarkers   = ros2_aruco_interfaces::msg::ArucoMarkers;

static double clamp(double v, double lo, double hi) {
  return std::max(lo, std::min(hi, v));
}
static bool isFinite(double v) { return std::isfinite(v); }

static double distXYZ(double x, double y, double z) {
  return std::sqrt(x*x + y*y + z*z);
}

static bool operateGripper(
  rclcpp::Node * node,
  const rclcpp_action::Client<GripperCommand>::SharedPtr & client,
  double pos,
  double effort = 10.0,
  std::chrono::milliseconds server_wait = 2000ms,
  std::chrono::milliseconds goal_wait   = 5000ms,
  std::chrono::milliseconds result_wait = 8000ms
)
{
  if (!client->wait_for_action_server(server_wait)) {
    RCLCPP_ERROR(node->get_logger(), "Gripper action server not available");
    return false;
  }

  GripperCommand::Goal goal;
  goal.command.position = pos;
  goal.command.max_effort = effort;

  rclcpp_action::Client<GripperCommand>::SendGoalOptions opt;
  opt.goal_response_callback =
    [node](const rclcpp_action::ClientGoalHandle<GripperCommand>::SharedPtr & gh) {
      if (!gh) RCLCPP_ERROR(node->get_logger(), "Gripper goal rejected (response cb)");
      else     RCLCPP_INFO (node->get_logger(), "Gripper goal accepted");
    };

  auto fut_goal = client->async_send_goal(goal, opt);

  auto ret = rclcpp::spin_until_future_complete(
    node->get_node_base_interface(), fut_goal, goal_wait);

  if (ret != rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(node->get_logger(), "Gripper send_goal timeout (spin_until_future_complete)");
    return false;
  }

  auto gh = fut_goal.get();
  if (!gh) {
    RCLCPP_ERROR(node->get_logger(), "Gripper goal rejected");
    return false;
  }

  auto fut_res = client->async_get_result(gh);
  ret = rclcpp::spin_until_future_complete(
    node->get_node_base_interface(), fut_res, result_wait);

  if (ret != rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(node->get_logger(), "Gripper result timeout (spin_until_future_complete)");
    return false;
  }

  auto wrapped = fut_res.get();
  if (wrapped.code != rclcpp_action::ResultCode::SUCCEEDED) {
    RCLCPP_ERROR(node->get_logger(), "Gripper failed (code=%d)", (int)wrapped.code);
    return false;
  }

  return true;
}

enum class FSM {
  HOME_INIT,
  WAIT_MARKER,
  CENTER_ON_MARKER,
  APPROACH_CONTINUOUS,
  GRASP,
  LIFT_HOME
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("real_pick_node");

  // Executor (TF/sub 콜백 처리용)
  auto exec = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  exec->add_node(node);
  std::thread spinner([&exec](){ exec->spin(); });

  auto tf_buffer  = std::make_unique<tf2_ros::Buffer>(node->get_clock());
  auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

  moveit::planning_interface::MoveGroupInterface arm(node, "arm");

  const std::string base_frame    = "link1";
  const std::string camera_frame  = "camera_link";
  const std::string markers_topic = "/aruco/markers";

  arm.setPoseReferenceFrame(base_frame);

  arm.setMaxVelocityScalingFactor(0.20);
  arm.setMaxAccelerationScalingFactor(0.20);
  arm.setPlanningTime(3.0);
  arm.setGoalPositionTolerance(0.01);
  arm.setGoalOrientationTolerance(3.14);

  auto gripper = rclcpp_action::create_client<GripperCommand>(
      node, "/gripper_controller/gripper_cmd");

  const double GRIP_OPEN  = 0.019;
  const double GRIP_CLOSE = 0.0;

  // ===== 파라미터/튜닝 =====
  const double max_tf_age_sec = 1.0;

  const double marker_z_min = 0.05;
  const double marker_z_max = 2.00;

  // 화면 중앙 정렬(헤드 조인트) 제어
  const double K_yaw   = 0.6;
  const double K_pitch = 0.6;
  const double max_step_rad = 0.12;

  node->declare_parameter<int>("yaw_cmd_sign", -1);
  node->declare_parameter<int>("pitch_cmd_sign", +1);

  const double yaw_tol_rad   = 3.0 * M_PI / 180.0;
  const double pitch_tol_rad = 3.0 * M_PI / 180.0;
  const int center_need = 5;
  int center_count = 0;

  const double j1_min = -M_PI, j1_max = M_PI;
  const double j2_min = -1.5,  j2_max = 1.5;

  // 접근: 베이스 기준으로 마커 중심을 계속 갱신하면서 "가상 목표점"으로 이동
  const double z_min  = 0.05;
  const double z_max  = 0.35;
  const double xy_max = 0.30;

  // ✅ “가상 목표점”: 마커 중심에서 x방향으로 약간 떨어진 지점(접근용)
  // (축 방향이 로봇마다 다르니 필요하면 +/- 바꿔야 함)
  const double approach_dx = 0.06;   // 접근하면서 유지할 x 오프셋(6cm)
  const double lift_dist   = 0.10;

  // ✅ 20cm 이내 진입하면 그리퍼 닫기
  const double close_dist_m = 0.20;

  // 접근을 한 번에 크게 움직이지 않도록 “스텝 제한”
  const double max_step_m = 0.05; // 5cm씩만 따라가기

  int fail_count = 0;
  const int max_fail = 3;

  std::mutex mk_mtx;
  ArucoMarkers latest;
  bool have_markers = false;

  auto sub = node->create_subscription<ArucoMarkers>(
    markers_topic, 10,
    [&](const ArucoMarkers::SharedPtr msg){
      std::lock_guard<std::mutex> lk(mk_mtx);
      latest = *msg;
      have_markers = true;
    }
  );

  auto selectClosestMarkerId = [&]() -> int {
    std::lock_guard<std::mutex> lk(mk_mtx);
    if (!have_markers) return -1;
    if (latest.marker_ids.size() != latest.poses.size()) return -1;
    if (latest.marker_ids.empty()) return -1;

    int best_id = -1;
    double best_z = 1e9;

    for (size_t i = 0; i < latest.marker_ids.size(); i++) {
      const double z = latest.poses[i].position.z; // camera frame pose라 가정(ros2_aruco 기본)
      if (!isFinite(z) || z < marker_z_min || z > marker_z_max) continue;
      if (z < best_z) {
        best_z = z;
        best_id = latest.marker_ids[i];
      }
    }
    return best_id;
  };

  auto getFreshTF = [&](const std::string& target, const std::string& source,
                        geometry_msgs::msg::TransformStamped& out) -> bool
  {
    try {
      if (!tf_buffer->canTransform(target, source, tf2::TimePointZero, 80ms))
        return false;

      auto t = tf_buffer->lookupTransform(target, source, tf2::TimePointZero);

      // stamp=0이면 age 체크 스킵(정적 TF 대비)
      if (!(t.header.stamp.sec == 0 && t.header.stamp.nanosec == 0)) {
        rclcpp::Time now = node->get_clock()->now();
        rclcpp::Time stamp = rclcpp::Time(t.header.stamp);
        const double age = (now - stamp).seconds();
        if (age < 0.0 || age > max_tf_age_sec) return false;
      }

      const auto &tr = t.transform.translation;
      if (!isFinite(tr.x) || !isFinite(tr.y) || !isFinite(tr.z)) return false;

      out = t;
      return true;
    } catch (...) {
      return false;
    }
  };

  auto stepToward = [&](double cur, double tgt) -> double {
    const double d = tgt - cur;
    if (std::abs(d) <= max_step_m) return tgt;
    return cur + (d > 0 ? max_step_m : -max_step_m);
  };

  FSM state = FSM::HOME_INIT;

  int target_id = -1;
  std::string target_frame;

  bool opened_on_detect = false;

  rclcpp::Rate rate(10);

  while (rclcpp::ok()) {
    switch (state) {

      case FSM::HOME_INIT: {
        RCLCPP_INFO(node->get_logger(), ">> HOME_INIT: go 'home', gripper open");
        arm.setNamedTarget("home");
        arm.move();
        rclcpp::sleep_for(500ms);

        (void)operateGripper(node.get(), gripper, GRIP_OPEN, 10.0);

        fail_count = 0;
        center_count = 0;
        target_id = -1;
        target_frame.clear();
        opened_on_detect = false;

        state = FSM::WAIT_MARKER;
        break;
      }

      case FSM::WAIT_MARKER: {
        const int best = selectClosestMarkerId();
        if (best < 0) {
          center_count = 0;
          opened_on_detect = false;
          RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 2000,
                               ">> WAIT_MARKER: no marker yet");
          break;
        }

        if (best != target_id) {
          target_id = best;
          target_frame = "aruco_marker_" + std::to_string(target_id);
          center_count = 0;
          opened_on_detect = false;
          RCLCPP_INFO(node->get_logger(), ">> Target marker set: ID=%d (%s)",
                      target_id, target_frame.c_str());
        }

        geometry_msgs::msg::TransformStamped t_cam_marker;
        if (!getFreshTF(camera_frame, target_frame, t_cam_marker)) {
          center_count = 0;
          break;
        }

        const double mx = t_cam_marker.transform.translation.x;
        const double my = t_cam_marker.transform.translation.y;
        const double mz = t_cam_marker.transform.translation.z;
        const double md = distXYZ(mx, my, mz);
        RCLCPP_INFO(node->get_logger(),
                    ">> MARKER(camera->%s): xyz=(%.3f, %.3f, %.3f) dist=%.3f m",
                    target_frame.c_str(), mx, my, mz, md);

        if (!opened_on_detect) {
          RCLCPP_INFO(node->get_logger(), ">> Marker detected! Open gripper.");
          (void)operateGripper(node.get(), gripper, GRIP_OPEN, 10.0);
          opened_on_detect = true;
        }

        state = FSM::CENTER_ON_MARKER;
        break;
      }

      case FSM::CENTER_ON_MARKER: {
        if (target_frame.empty()) {
          state = FSM::WAIT_MARKER;
          break;
        }

        geometry_msgs::msg::TransformStamped t_cam_marker;
        if (!getFreshTF(camera_frame, target_frame, t_cam_marker)) {
          center_count = 0;
          state = FSM::WAIT_MARKER;
          break;
        }

        const double x = t_cam_marker.transform.translation.x;
        const double y = t_cam_marker.transform.translation.y;
        const double z = t_cam_marker.transform.translation.z;
        const double d = distXYZ(x, y, z);

        RCLCPP_INFO(node->get_logger(),
                    ">> CENTER: cam xyz=(%.3f, %.3f, %.3f) dist=%.3f m",
                    x, y, z, d);

        // ✅ 20cm 이내면 즉시 GRASP (요구사항)
        if (d <= close_dist_m) {
          RCLCPP_INFO(node->get_logger(),
                      ">> CLOSE CONDITION: %.3f m <= %.3f m => GRASP",
                      d, close_dist_m);
          state = FSM::GRASP;
          break;
        }

        // 화면 중심 정렬(헤드 yaw/pitch)
        const double yaw_err   = std::atan2(x, z);
        const double pitch_err = std::atan2(-y, z);

        if (std::abs(yaw_err) < yaw_tol_rad && std::abs(pitch_err) < pitch_tol_rad) {
          center_count++;
          RCLCPP_INFO(node->get_logger(), ">> CENTER OK %d/%d", center_count, center_need);
        } else {
          center_count = 0;
        }

        // 헤드 조인트 갱신
        std::vector<double> joints = arm.getCurrentJointValues();
        if (joints.size() < 2) {
          state = FSM::WAIT_MARKER;
          break;
        }

        int yaw_cmd_sign = -1;
        int pitch_cmd_sign = +1;
        node->get_parameter("yaw_cmd_sign", yaw_cmd_sign);
        node->get_parameter("pitch_cmd_sign", pitch_cmd_sign);
        yaw_cmd_sign   = (yaw_cmd_sign >= 0) ? 1 : -1;
        pitch_cmd_sign = (pitch_cmd_sign >= 0) ? 1 : -1;

        double dyaw   = clamp(yaw_cmd_sign * K_yaw * yaw_err,   -max_step_rad, max_step_rad);
        double dpitch = clamp(-pitch_cmd_sign * K_pitch * pitch_err, -max_step_rad, max_step_rad);

        joints[0] = clamp(joints[0] + dyaw,   j1_min, j1_max);
        joints[1] = clamp(joints[1] + dpitch, j2_min, j2_max);

        arm.setStartStateToCurrentState();
        arm.setJointValueTarget(joints);

        auto res = arm.move();
        if (res != moveit::core::MoveItErrorCode::SUCCESS) {
          fail_count++;
          RCLCPP_WARN(node->get_logger(), "CENTER move failed (%d/%d)", fail_count, max_fail);
          if (fail_count >= max_fail) state = FSM::HOME_INIT;
          else state = FSM::WAIT_MARKER;
          break;
        } else {
          fail_count = 0;
        }

        // 중심 정렬이 어느 정도 되면 접근 단계로
        if (center_count >= center_need) {
          state = FSM::APPROACH_CONTINUOUS;
          RCLCPP_INFO(node->get_logger(), ">> CENTER DONE -> APPROACH_CONTINUOUS");
        }

        break;
      }

      case FSM::APPROACH_CONTINUOUS: {
        if (target_frame.empty()) {
          state = FSM::WAIT_MARKER;
          break;
        }

        // 1) 카메라 거리로 “20cm 이내” 체크 (지속 접근 중에도 계속 체크)
        geometry_msgs::msg::TransformStamped t_cam_marker;
        if (!getFreshTF(camera_frame, target_frame, t_cam_marker)) {
          state = FSM::WAIT_MARKER;
          break;
        }
        const double cx = t_cam_marker.transform.translation.x;
        const double cy = t_cam_marker.transform.translation.y;
        const double cz = t_cam_marker.transform.translation.z;
        const double cd = distXYZ(cx, cy, cz);

        RCLCPP_INFO(node->get_logger(),
                    ">> APPROACH: cam dist=%.3f m (close<=%.3f)",
                    cd, close_dist_m);

        if (cd <= close_dist_m) {
          state = FSM::GRASP;
          break;
        }

        // 2) 베이스 기준 마커 위치로 “가상 목표점”을 매 루프 갱신
        geometry_msgs::msg::TransformStamped t_base_marker;
        if (!getFreshTF(base_frame, target_frame, t_base_marker)) {
          state = FSM::WAIT_MARKER;
          break;
        }

        const double bx = t_base_marker.transform.translation.x;
        const double by = t_base_marker.transform.translation.y;
        const double bz = t_base_marker.transform.translation.z;

        // 가상 목표점: marker 중심 + 접근 오프셋(approach_dx) + z 클램프
        // (approach_dx 부호는 로봇 축에 따라 바뀔 수 있음)
        const double goal_x = clamp(bx + approach_dx, -xy_max, xy_max);
        const double goal_y = clamp(by,             -xy_max, xy_max);
        const double goal_z = clamp(bz,              z_min,  z_max);

        // 3) 한 번에 크게 안 움직이도록 EE 현재 위치에서 스텝 제한
        auto ee_now = arm.getCurrentPose().pose;

        const double step_x = stepToward(ee_now.position.x, goal_x);
        const double step_y = stepToward(ee_now.position.y, goal_y);
        const double step_z = stepToward(ee_now.position.z, goal_z);

        geometry_msgs::msg::Pose tgt = ee_now;
        tgt.position.x = step_x;
        tgt.position.y = step_y;
        tgt.position.z = step_z;

        // orientation 유지
        // tgt.orientation = ee_now.orientation;

        RCLCPP_INFO(node->get_logger(),
                    ">> APPROACH target(base): (%.3f %.3f %.3f) step->(%.3f %.3f %.3f)",
                    goal_x, goal_y, goal_z, step_x, step_y, step_z);

        arm.setStartStateToCurrentState();
        arm.setPoseTarget(tgt);

        auto res = arm.move();
        arm.clearPoseTargets();

        if (res != moveit::core::MoveItErrorCode::SUCCESS) {
          fail_count++;
          RCLCPP_WARN(node->get_logger(), "APPROACH move failed (%d/%d)", fail_count, max_fail);
          if (fail_count >= max_fail) state = FSM::HOME_INIT;
          else state = FSM::CENTER_ON_MARKER; // 다시 중심부터
          break;
        } else {
          fail_count = 0;
        }

        break;
      }

      case FSM::GRASP: {
        RCLCPP_INFO(node->get_logger(), ">> GRASP: close gripper NOW");
        (void)operateGripper(node.get(), gripper, GRIP_CLOSE, 30.0);
        state = FSM::LIFT_HOME;
        break;
      }

      case FSM::LIFT_HOME: {
        RCLCPP_INFO(node->get_logger(), ">> LIFT_HOME: lift and home");
        auto ee = arm.getCurrentPose().pose;

        geometry_msgs::msg::Pose lift_pose = ee;
        lift_pose.position.z = clamp(lift_pose.position.z + lift_dist, z_min, z_max);

        arm.setStartStateToCurrentState();
        arm.setPoseTarget(lift_pose);
        arm.move();
        arm.clearPoseTargets();

        arm.setNamedTarget("home");
        arm.move();

        center_count = 0;
        fail_count = 0;
        opened_on_detect = false;
        state = FSM::WAIT_MARKER;
        break;
      }
    }

    rate.sleep();
  }

  rclcpp::shutdown();
  spinner.join();
  return 0;
}