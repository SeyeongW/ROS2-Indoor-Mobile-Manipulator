// omx_real_picker_tf_stop_final.cpp

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <control_msgs/action/gripper_command.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/exceptions.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <chrono>
#include <cmath>
#include <deque>
#include <vector>
#include <algorithm>
#include <memory>

using namespace std::chrono_literals;
using GripperCommand = control_msgs::action::GripperCommand;

static bool isFinite(double v){ return std::isfinite(v); }

static double medianPush(std::deque<double>& dq, double v, size_t N){
  dq.push_back(v);
  if(dq.size() > N) dq.pop_front();
  std::vector<double> tmp(dq.begin(), dq.end());
  std::sort(tmp.begin(), tmp.end());
  return tmp[tmp.size()/2];
}

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
  goal.command.position   = pos;
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

class OmxPickerFinal : public rclcpp::Node
{
public:
  explicit OmxPickerFinal(const rclcpp::NodeOptions& opts = rclcpp::NodeOptions())
  : Node("omx_real_picker_tf_stop_final", opts)
  {}

  // 생성자에서 shared_from_this()를 쓰지 않기 위해 init()로 분리
  void init(const rclcpp::Node::SharedPtr& move_group_arm_node)
  {
    // ------------------------------------------------------------
    // use_sim_time (중복 declare 방지)
    // ------------------------------------------------------------
    if(!this->has_parameter("use_sim_time")){
      this->declare_parameter<bool>("use_sim_time", true);
    }
    bool use_sim_time=true;
    this->get_parameter("use_sim_time", use_sim_time);
    RCLCPP_INFO(get_logger(), "use_sim_time=%s", use_sim_time ? "true" : "false");

    // ------------------------------------------------------------
    // Params: 기존 값 유지
    // ------------------------------------------------------------
    base_frame_   = this->declare_parameter<std::string>("base_frame", "link1");
    marker_frame_ = this->declare_parameter<std::string>("marker_frame", "aruco_marker_23");

    // MoveIt params (기존 값 유지)
    vel_scale_ = this->declare_parameter<double>("vel_scale", 0.15);
    acc_scale_ = this->declare_parameter<double>("acc_scale", 0.15);
    planning_time_ = this->declare_parameter<double>("planning_time", 5.0);
    pos_tol_ = this->declare_parameter<double>("pos_tol", 0.03);
    ori_tol_ = this->declare_parameter<double>("ori_tol", 3.14);

    // Gripper params (기존 값 유지)
    GRIP_OPEN_   = this->declare_parameter<double>("grip_open", 0.019);
    GRIP_CLOSE_  = this->declare_parameter<double>("grip_close", 0.0);
    GRIP_EFFORT_ = this->declare_parameter<double>("grip_effort", 30.0);

    // SEARCH waypoints (기존 값 유지)
    waypoints_ = {
      { 0.00, -0.20,  0.20,  0.80},
      { 1.00, -0.20,  0.20,  0.80},
      {-1.00, -0.20,  0.20,  0.80},
      { 0.00, -0.60,  0.30,  1.20}
    };

    // Approach params (기존 값 유지)
    hover_offset_ = this->declare_parameter<double>("hover_offset", 0.15);
    step_         = this->declare_parameter<double>("step", 0.03);
    final_min_z_  = this->declare_parameter<double>("final_min_z", 0.04);

    // TF freshness (기존 + 성공 방식 결합)
    max_tf_age_sec_      = this->declare_parameter<double>("max_tf_age_sec", 0.7);
    filtering_time_sec_  = this->declare_parameter<double>("filtering_time_sec", 2.0);

    // stop-based (성공 방식)
    stop_time_threshold_sec_ = this->declare_parameter<double>("stop_time_threshold_sec", 2.0);
    distance_threshold_m_    = this->declare_parameter<double>("distance_threshold_m", 0.01);

    // median filter (기존)
    FILTER_N_ = (size_t)this->declare_parameter<int>("median_filter_N", 7);

    // ------------------------------------------------------------
    // TF
    // ------------------------------------------------------------
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // ------------------------------------------------------------
    // MoveIt arm (중요: move_group_arm_node로 생성)
    // ------------------------------------------------------------
    arm_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(move_group_arm_node, "arm");
    arm_->setPoseReferenceFrame(base_frame_);
    arm_->setMaxVelocityScalingFactor(vel_scale_);
    arm_->setMaxAccelerationScalingFactor(acc_scale_);
    arm_->setPlanningTime(planning_time_);
    arm_->setGoalPositionTolerance(pos_tol_);
    arm_->setGoalOrientationTolerance(ori_tol_);

    RCLCPP_INFO(get_logger(), "Planning frame: %s", arm_->getPlanningFrame().c_str());
    RCLCPP_INFO(get_logger(), "Pose reference frame: %s", base_frame_.c_str());

    // ------------------------------------------------------------
    // Gripper action client (이 노드로 생성)
    // ------------------------------------------------------------
    gripper_ = rclcpp_action::create_client<GripperCommand>(
      this->shared_from_this(), "/gripper_controller/gripper_cmd");

    // ------------------------------------------------------------
    // 초기 동작 (기존 흐름 유지)
    // ------------------------------------------------------------
    RCLCPP_INFO(get_logger(), ">> HOME");
    arm_->setNamedTarget("home");
    arm_->move();
    rclcpp::sleep_for(600ms);

    operateGripperBlocking(this->shared_from_this(), gripper_, GRIP_OPEN_, GRIP_EFFORT_);

    // ------------------------------------------------------------
    // state init
    // ------------------------------------------------------------
    state_ = FSM::SEARCH;
    wp_idx_ = 0;
    busy_ = false;

    resetStabilize();

    locked_valid_ = false;
    current_offset_ = hover_offset_;

    // timer tick
    timer_ = this->create_wall_timer(200ms, std::bind(&OmxPickerFinal::tick, this));
  }

private:
  enum class FSM { SEARCH, STABILIZE, APPROACH, GRASP, LIFT_AND_HOME };

  // ---- 패치 2: 워밍업 포함 리셋 ----
  void resetStabilize()
  {
    fx_.clear(); fy_.clear(); fz_.clear();
    warmup_count_ = 0;
    have_prev_for_stop_ = false;
    stop_start_time_ = this->get_clock()->now();
  }

  // ---- TF 읽기: 예외/신선도 필터/NaN 방어 ----
  bool getMarkerFromTF(double& raw_x, double& raw_y, double& raw_z,
                       double& filt_x, double& filt_y, double& filt_z)
  {
    geometry_msgs::msg::TransformStamped tf_msg;
    try{
      tf_msg = tf_buffer_->lookupTransform(base_frame_, marker_frame_, tf2::TimePointZero);
    }catch(const tf2::TransformException&){
      return false;
    }

    const rclcpp::Time now = this->get_clock()->now();
    if(now.nanoseconds() == 0) return false; // sim time /clock 아직

    const rclcpp::Time st(tf_msg.header.stamp);
    const double age = (now - st).seconds();

    // 둘 다 적용(네 기존 + 성공 방식)
    if(age > max_tf_age_sec_) return false;
    if(age > filtering_time_sec_) return false;

    raw_x = tf_msg.transform.translation.x;
    raw_y = tf_msg.transform.translation.y;
    raw_z = tf_msg.transform.translation.z;

    if(!isFinite(raw_x)||!isFinite(raw_y)||!isFinite(raw_z)) return false;

    filt_x = medianPush(fx_, raw_x, FILTER_N_);
    filt_y = medianPush(fy_, raw_y, FILTER_N_);
    filt_z = medianPush(fz_, raw_z, FILTER_N_);

    // 워밍업 카운트(패치 2)
    if(warmup_count_ < (int)FILTER_N_) warmup_count_++;

    return true;
  }

  // ---- 패치 2: 워밍업 후에만 stop 판정 ----
  bool isStoppedLongEnough(double x, double y, double z)
  {
    if(warmup_count_ < (int)FILTER_N_) {
      // 필터 안정화 전에는 정지 판정 안 함
      return false;
    }

    const rclcpp::Time now = this->get_clock()->now();

    if(!have_prev_for_stop_){
      prev_x_ = x; prev_y_ = y; prev_z_ = z;
      stop_start_time_ = now;
      have_prev_for_stop_ = true;
      return false;
    }

    const double dx = x - prev_x_;
    const double dy = y - prev_y_;
    const double dz = z - prev_z_;
    const double dist = std::sqrt(dx*dx + dy*dy + dz*dz);

    if(dist < distance_threshold_m_){
      const double stopped_sec = (now - stop_start_time_).seconds();
      return (stopped_sec >= stop_time_threshold_sec_);
    }else{
      prev_x_ = x; prev_y_ = y; prev_z_ = z;
      stop_start_time_ = now;
      return false;
    }
  }

  void setPoseXYZKeepOri(double x, double y, double z)
  {
    geometry_msgs::msg::PoseStamped tgt;
    tgt.header.frame_id = base_frame_;
    tgt.header.stamp = this->get_clock()->now();

    tgt.pose.position.x = x;
    tgt.pose.position.y = y;
    tgt.pose.position.z = z;

    // 너 방식 유지(현재 orientation 고정)
    tgt.pose.orientation = arm_->getCurrentPose().pose.orientation;

    arm_->setPoseTarget(tgt);
    arm_->move();
  }

  void tick()
  {
    if(busy_) return; // 재진입 방지

    double rx=0, ry=0, rz=0;
    double mx=0, my=0, mz=0;

    const bool visible = getMarkerFromTF(rx,ry,rz, mx,my,mz);

    switch(state_){
      case FSM::SEARCH: {
        // SEARCH에서는 안정화 상태 리셋
        resetStabilize();
        locked_valid_ = false;

        if(visible){
          RCLCPP_INFO(get_logger(), ">> Seen -> STABILIZE");
          state_ = FSM::STABILIZE;
          break;
        }

        RCLCPP_INFO(get_logger(), ">> SEARCH waypoint %d", wp_idx_);
        arm_->setJointValueTarget(waypoints_[wp_idx_]);
        arm_->move();
        wp_idx_ = (wp_idx_ + 1) % (int)waypoints_.size();
      } break;

      case FSM::STABILIZE: {
        if(!visible){
          RCLCPP_WARN(get_logger(), ">> Lost -> SEARCH");
          state_ = FSM::SEARCH;
          break;
        }

        const bool stopped = isStoppedLongEnough(mx,my,mz);
        RCLCPP_INFO(get_logger(), ">> STABILIZE warmup=%d/%zu stop=%s (%.3f %.3f %.3f)",
                    warmup_count_, FILTER_N_, stopped ? "YES" : "no", mx,my,mz);

        if(stopped){
          // ---- 패치 1: locked_target 고정 ----
          locked_x_ = mx;
          locked_y_ = my;
          locked_z_ = std::max(final_min_z_, mz); // 너의 final_min_z 기준 유지
          locked_valid_ = true;

          current_offset_ = hover_offset_;
          RCLCPP_INFO(get_logger(), ">> Lock target (%.3f %.3f %.3f) -> APPROACH",
                      locked_x_, locked_y_, locked_z_);
          state_ = FSM::APPROACH;
        }
      } break;

      case FSM::APPROACH: {
        // 접근 중에는 "locked"만 사용 (패치 1)
        if(!locked_valid_){
          RCLCPP_WARN(get_logger(), ">> No locked target -> SEARCH");
          state_ = FSM::SEARCH;
          break;
        }

        // 접근 중 TF가 잠깐 끊겨도 locked로 계속 갈 수도 있고,
        // 네가 원하는게 "끊기면 멈춰"면 아래처럼 처리:
        if(!visible){
          RCLCPP_WARN(get_logger(), ">> TF lost during APPROACH -> STABILIZE");
          resetStabilize();
          state_ = FSM::STABILIZE;
          break;
        }

        const double target_z = std::max(final_min_z_, locked_z_ + current_offset_);

        RCLCPP_INFO(get_logger(), ">> APPROACH off=%.3f tgt=(%.3f %.3f %.3f) [LOCKED]",
                    current_offset_, locked_x_, locked_y_, target_z);

        setPoseXYZKeepOri(locked_x_, locked_y_, target_z);

        current_offset_ = std::max(0.0, current_offset_ - step_);

        if(current_offset_ <= 1e-4){
          RCLCPP_INFO(get_logger(), ">> Reached -> GRASP");
          state_ = FSM::GRASP;
        }
      } break;

      case FSM::GRASP: {
        busy_ = true;
        if(!operateGripperBlocking(this->shared_from_this(), gripper_, GRIP_CLOSE_, GRIP_EFFORT_)){
          RCLCPP_ERROR(get_logger(), "Gripper close failed");
        }
        busy_ = false;
        state_ = FSM::LIFT_AND_HOME;
      } break;

      case FSM::LIFT_AND_HOME: {
        busy_ = true;

        // lift
        auto cur = arm_->getCurrentPose();
        cur.header.frame_id = base_frame_;
        cur.pose.position.z += 0.10;
        arm_->setPoseTarget(cur);
        arm_->move();

        // home
        arm_->setNamedTarget("home");
        arm_->move();

        // open
        operateGripperBlocking(this->shared_from_this(), gripper_, GRIP_OPEN_, GRIP_EFFORT_);

        busy_ = false;

        // 다음 싸이클
        locked_valid_ = false;
        state_ = FSM::SEARCH;
      } break;
    }
  }

private:
  // frames
  std::string base_frame_;
  std::string marker_frame_;

  // MoveIt
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_;
  double vel_scale_{0.15}, acc_scale_{0.15}, planning_time_{5.0}, pos_tol_{0.03}, ori_tol_{3.14};

  // Gripper
  rclcpp_action::Client<GripperCommand>::SharedPtr gripper_;
  double GRIP_OPEN_{0.019}, GRIP_CLOSE_{0.0}, GRIP_EFFORT_{30.0};

  // TF
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;

  // Search
  std::vector<std::vector<double>> waypoints_;
  int wp_idx_{0};

  // Approach params
  double hover_offset_{0.15}, step_{0.03}, final_min_z_{0.04};
  double current_offset_{0.15};

  // TF freshness
  double max_tf_age_sec_{0.7};
  double filtering_time_sec_{2.0};

  // Stop-based
  double stop_time_threshold_sec_{2.0};
  double distance_threshold_m_{0.01};

  // Median filter
  size_t FILTER_N_{7};
  std::deque<double> fx_, fy_, fz_;
  int warmup_count_{0};

  // Stop detect state
  bool have_prev_for_stop_{false};
  double prev_x_{0}, prev_y_{0}, prev_z_{0};
  rclcpp::Time stop_start_time_;

  // Locked target (패치 1)
  bool locked_valid_{false};
  double locked_x_{0}, locked_y_{0}, locked_z_{0};

  // FSM
  enum class FSM { SEARCH, STABILIZE, APPROACH, GRASP, LIFT_AND_HOME };
  FSM state_{FSM::SEARCH};

  bool busy_{false};
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  // move_group_arm_node를 성공 예제처럼 분리 (안정성 ↑)
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);

  auto move_group_arm_node = rclcpp::Node::make_shared("move_group_arm_node", node_options);

  auto picker = std::make_shared<OmxPickerFinal>(node_options);
  picker->init(move_group_arm_node);

  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(picker);
  exec.add_node(move_group_arm_node);
  exec.spin();

  rclcpp::shutdown();
  return 0;
}