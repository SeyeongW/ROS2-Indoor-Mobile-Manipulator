#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <control_msgs/action/gripper_command.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/move_it_error_codes.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/exceptions.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <chrono>
#include <cmath>
#include <deque>
#include <vector>
#include <algorithm>
#include <memory>
#include <string>

using namespace std::chrono_literals;
using GripperCommand = control_msgs::action::GripperCommand;

static bool isMoveSuccess(const moveit::core::MoveItErrorCode & code)
{
  return code.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
}

static double medianPush(std::deque<double> & dq, double v, size_t n)
{
  dq.push_back(v);
  if (dq.size() > n) {
    dq.pop_front();
  }
  std::vector<double> tmp(dq.begin(), dq.end());
  std::sort(tmp.begin(), tmp.end());
  return tmp[tmp.size() / 2];
}

class BusyGuard
{
public:
  explicit BusyGuard(bool & busy) : busy_(busy) {busy_ = true;}
  ~BusyGuard() {busy_ = false;}

private:
  bool & busy_;
};

class OmxPickerFinal : public rclcpp::Node
{
public:
  explicit OmxPickerFinal(const rclcpp::NodeOptions & opts = rclcpp::NodeOptions())
  : Node("omx_real_picker_tf_stop_final", opts)
  {
  }

  void init(const rclcpp::Node::SharedPtr & move_group_arm_node)
  {
    base_frame_ = this->declare_parameter<std::string>("base_frame", "link1");
    marker_frame_ = this->declare_parameter<std::string>("marker_frame", "aruco_marker_23");

    vel_scale_ = this->declare_parameter<double>("vel_scale", 0.15);
    acc_scale_ = this->declare_parameter<double>("acc_scale", 0.15);
    grip_open_ = this->declare_parameter<double>("grip_open", 0.019);
    grip_close_ = this->declare_parameter<double>("grip_close", 0.0);
    grip_effort_ = this->declare_parameter<double>("grip_effort", 30.0);

    hover_offset_ = this->declare_parameter<double>("hover_offset", 0.15);
    step_ = this->declare_parameter<double>("step", 0.03);
    final_min_z_ = this->declare_parameter<double>("final_min_z", 0.04);
    max_tf_age_sec_ = this->declare_parameter<double>("max_tf_age_sec", 0.7);
    stop_time_threshold_sec_ = this->declare_parameter<double>("stop_time_threshold_sec", 2.0);
    distance_threshold_m_ = this->declare_parameter<double>("distance_threshold_m", 0.01);
    filter_n_ = static_cast<size_t>(std::max(1, this->declare_parameter<int>("median_filter_N", 7)));

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    arm_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(move_group_arm_node, "arm");
    arm_->setPoseReferenceFrame(base_frame_);
    arm_->setMaxVelocityScalingFactor(vel_scale_);
    arm_->setMaxAccelerationScalingFactor(acc_scale_);

    gripper_ = rclcpp_action::create_client<GripperCommand>(this, "/gripper_controller/gripper_cmd");

    waypoints_ = {
      {0.0, -0.2, 0.2, 0.8},
      {1.0, -0.2, 0.2, 0.8},
      {-1.0, -0.2, 0.2, 0.8},
      {0.0, -0.6, 0.3, 1.2}
    };

    RCLCPP_INFO(get_logger(), ">> Initializing: Move to HOME");
    arm_->setNamedTarget("home");
    executeArmMove("initial home");
    operateGripperBlocking(grip_open_, grip_effort_);

    state_ = FSM::SEARCH;
    timer_ = this->create_wall_timer(200ms, std::bind(&OmxPickerFinal::tick, this));
  }

private:
  enum class FSM {SEARCH, STABILIZE, APPROACH, GRASP, LIFT_AND_HOME};

  bool executeArmMove(const std::string & context)
  {
    const auto result = arm_->move();
    arm_->clearPoseTargets();

    if (!isMoveSuccess(result)) {
      RCLCPP_ERROR(get_logger(), "Arm move failed: %s (code=%d)", context.c_str(), result.val);
      return false;
    }
    return true;
  }

  bool operateGripperBlocking(double pos, double effort)
  {
    if (!gripper_->wait_for_action_server(1s)) {
      RCLCPP_ERROR(get_logger(), "Gripper action server not available");
      return false;
    }

    GripperCommand::Goal goal;
    goal.command.position = pos;
    goal.command.max_effort = effort;

    auto fut_goal = gripper_->async_send_goal(goal);
    if (fut_goal.wait_for(2s) != std::future_status::ready) {
      RCLCPP_ERROR(get_logger(), "Gripper goal send timeout");
      return false;
    }

    auto gh = fut_goal.get();
    if (!gh) {
      RCLCPP_ERROR(get_logger(), "Gripper goal rejected");
      return false;
    }

    auto fut_res = gripper_->async_get_result(gh);
    if (fut_res.wait_for(4s) != std::future_status::ready) {
      RCLCPP_ERROR(get_logger(), "Gripper result timeout");
      return false;
    }

    auto wrapped = fut_res.get();
    if (wrapped.code != rclcpp_action::ResultCode::SUCCEEDED) {
      RCLCPP_ERROR(get_logger(), "Gripper action failed (result code=%d)", static_cast<int>(wrapped.code));
      return false;
    }

    return true;
  }

  void resetStabilize()
  {
    fx_.clear();
    fy_.clear();
    fz_.clear();
    warmup_count_ = 0;
    have_prev_for_stop_ = false;
    stop_start_time_ = this->get_clock()->now();
  }

  bool getMarkerFromTF(double & mx, double & my, double & mz)
  {
    geometry_msgs::msg::TransformStamped tf_msg;
    try {
      tf_msg = tf_buffer_->lookupTransform(base_frame_, marker_frame_, tf2::TimePointZero, 100ms);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_DEBUG(get_logger(), "TF lookup failed: %s", ex.what());
      return false;
    }

    if (tf_msg.header.stamp.sec != 0 || tf_msg.header.stamp.nanosec != 0) {
      const auto now = this->get_clock()->now();
      const double age = (now - tf_msg.header.stamp).seconds();
      if (age < 0.0 || age > max_tf_age_sec_) {
        return false;
      }
    }

    const double rx = tf_msg.transform.translation.x;
    const double ry = tf_msg.transform.translation.y;
    const double rz = tf_msg.transform.translation.z;

    if (!std::isfinite(rx) || !std::isfinite(ry) || !std::isfinite(rz)) {
      return false;
    }

    mx = medianPush(fx_, rx, filter_n_);
    my = medianPush(fy_, ry, filter_n_);
    mz = medianPush(fz_, rz, filter_n_);

    if (warmup_count_ < static_cast<int>(filter_n_)) {
      warmup_count_++;
    }

    return true;
  }

  bool isStopped(double x, double y, double z)
  {
    if (warmup_count_ < static_cast<int>(filter_n_)) {
      return false;
    }

    const auto now = this->get_clock()->now();
    if (!have_prev_for_stop_) {
      prev_x_ = x;
      prev_y_ = y;
      prev_z_ = z;
      stop_start_time_ = now;
      have_prev_for_stop_ = true;
      return false;
    }

    const double dist = std::sqrt(
      std::pow(x - prev_x_, 2) + std::pow(y - prev_y_, 2) + std::pow(z - prev_z_, 2));

    if (dist < distance_threshold_m_) {
      return (now - stop_start_time_).seconds() >= stop_time_threshold_sec_;
    }

    prev_x_ = x;
    prev_y_ = y;
    prev_z_ = z;
    stop_start_time_ = now;
    return false;
  }

  bool setPoseKeepOri(double x, double y, double z)
  {
    geometry_msgs::msg::PoseStamped tgt;
    tgt.header.frame_id = base_frame_;
    tgt.header.stamp = this->get_clock()->now();
    tgt.pose.position.x = x;
    tgt.pose.position.y = y;
    tgt.pose.position.z = z;
    tgt.pose.orientation = arm_->getCurrentPose().pose.orientation;

    arm_->setPoseTarget(tgt);
    return executeArmMove("approach step");
  }

  void tick()
  {
    if (busy_) {
      return;
    }

    double mx = 0.0;
    double my = 0.0;
    double mz = 0.0;
    const bool visible = getMarkerFromTF(mx, my, mz);

    switch (state_) {
      case FSM::SEARCH: {
        resetStabilize();
        if (visible) {
          RCLCPP_INFO(get_logger(), ">> Marker spotted -> STABILIZE");
          state_ = FSM::STABILIZE;
          break;
        }

        BusyGuard guard(busy_);
        RCLCPP_INFO(get_logger(), ">> SEARCH: moving to waypoint %d", wp_idx_);
        arm_->setJointValueTarget(waypoints_[wp_idx_]);
        executeArmMove("search");
        wp_idx_ = (wp_idx_ + 1) % static_cast<int>(waypoints_.size());
        break;
      }

      case FSM::STABILIZE:
        if (!visible) {
          state_ = FSM::SEARCH;
          break;
        }

        if (isStopped(mx, my, mz)) {
          locked_x_ = mx;
          locked_y_ = my;
          locked_z_ = std::max(final_min_z_, mz);
          locked_valid_ = true;
          current_offset_ = hover_offset_;
          state_ = FSM::APPROACH;
          RCLCPP_INFO(get_logger(), ">> Target locked (%.3f, %.3f, %.3f)", locked_x_, locked_y_, locked_z_);
        }
        break;

      case FSM::APPROACH: {
        if (!visible || !locked_valid_) {
          RCLCPP_WARN(get_logger(), ">> Lost target during approach -> STABILIZE");
          resetStabilize();
          state_ = FSM::STABILIZE;
          break;
        }

        BusyGuard guard(busy_);
        const double target_z = std::max(final_min_z_, locked_z_ + current_offset_);
        if (setPoseKeepOri(locked_x_, locked_y_, target_z)) {
          current_offset_ -= step_;
          if (current_offset_ <= 0.0) {
            state_ = FSM::GRASP;
          }
        } else {
          state_ = FSM::STABILIZE;
        }
        break;
      }

      case FSM::GRASP: {
        BusyGuard guard(busy_);
        RCLCPP_INFO(get_logger(), ">> GRASPING...");
        if (!operateGripperBlocking(grip_close_, grip_effort_)) {
          RCLCPP_ERROR(get_logger(), "Gripper close failed");
          state_ = FSM::STABILIZE;
          break;
        }
        state_ = FSM::LIFT_AND_HOME;
        break;
      }

      case FSM::LIFT_AND_HOME: {
        BusyGuard guard(busy_);

        auto cur = arm_->getCurrentPose();
        cur.pose.position.z += 0.10;
        arm_->setPoseTarget(cur);
        executeArmMove("lift");

        arm_->setNamedTarget("home");
        executeArmMove("home");

        operateGripperBlocking(grip_open_, grip_effort_);

        locked_valid_ = false;
        state_ = FSM::SEARCH;
        break;
      }
    }
  }

  std::string base_frame_;
  std::string marker_frame_;

  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_;
  rclcpp_action::Client<GripperCommand>::SharedPtr gripper_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::vector<std::vector<double>> waypoints_;
  int wp_idx_ {0};

  double vel_scale_ {0.15};
  double acc_scale_ {0.15};
  double grip_open_ {0.019};
  double grip_close_ {0.0};
  double grip_effort_ {30.0};

  double hover_offset_ {0.15};
  double step_ {0.03};
  double final_min_z_ {0.04};
  double current_offset_ {0.15};

  double max_tf_age_sec_ {0.7};
  double stop_time_threshold_sec_ {2.0};
  double distance_threshold_m_ {0.01};

  size_t filter_n_ {7};
  std::deque<double> fx_;
  std::deque<double> fy_;
  std::deque<double> fz_;
  int warmup_count_ {0};

  bool have_prev_for_stop_ {false};
  double prev_x_ {0.0};
  double prev_y_ {0.0};
  double prev_z_ {0.0};
  rclcpp::Time stop_start_time_;

  bool locked_valid_ {false};
  double locked_x_ {0.0};
  double locked_y_ {0.0};
  double locked_z_ {0.0};

  bool busy_ {false};
  FSM state_ {FSM::SEARCH};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);

  auto move_group_node = rclcpp::Node::make_shared("move_group_arm_node", node_options);
  auto picker = std::make_shared<OmxPickerFinal>(node_options);
  picker->init(move_group_node);

  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(picker);
  exec.add_node(move_group_node);
  exec.spin();

  rclcpp::shutdown();
  return 0;
}
