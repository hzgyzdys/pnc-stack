// Copyright (c) 2026 Hu Zhiguo. MIT License.
//
// ROS 2 wrapper around pnc_core tracking controllers.
//
// Subscribes:  odom (nav_msgs/Odometry), plan (nav_msgs/Path)
// Publishes:   cmd_vel (geometry_msgs/Twist), ~/cross_track_error (std_msgs/Float64)
//
// The algorithm itself lives in pnc_core; this node only converts messages,
// runs the control timer and enforces safety timeouts.

#include <chrono>
#include <cmath>
#include <cstddef>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "pnc_core/math_utils.hpp"
#include "pnc_core/path_utils.hpp"
#include "pnc_core/pure_pursuit.hpp"
#include "pnc_core/vehicle_model.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

namespace pnc_control {

namespace {

double yawFromQuaternion(double x, double y, double z, double w) {
  return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
}

}  // namespace

class TrackingControllerNode : public rclcpp::Node {
 public:
  explicit TrackingControllerNode(const rclcpp::NodeOptions& options)
      : Node("tracking_controller", options) {
    // Vehicle limits.
    vehicle_.wheelbase = declare_parameter("vehicle.wheelbase", vehicle_.wheelbase);
    vehicle_.max_steer = declare_parameter("vehicle.max_steer", vehicle_.max_steer);
    vehicle_.max_steer_rate = declare_parameter("vehicle.max_steer_rate", vehicle_.max_steer_rate);
    vehicle_.max_accel = declare_parameter("vehicle.max_accel", vehicle_.max_accel);
    vehicle_.max_decel = declare_parameter("vehicle.max_decel", vehicle_.max_decel);
    vehicle_.max_speed = declare_parameter("vehicle.max_speed", vehicle_.max_speed);

    // Node behaviour.
    const double rate_hz = declare_parameter("control_rate_hz", 50.0);
    reference_speed_ = declare_parameter("reference_speed", 1.0);
    odom_timeout_s_ = declare_parameter("odom_timeout_s", 0.5);
    goal_tolerance_ = declare_parameter("goal_tolerance", 0.2);
    const std::string controller_type =
        declare_parameter<std::string>("controller.type", "pure_pursuit");

    controller_ = makeController(controller_type);
    model_ = std::make_unique<pnc_core::KinematicBicycle>(vehicle_);
    dt_ = 1.0 / rate_hz;

    // Latched plan, so a late-joining controller still receives the last path.
    const auto plan_qos = rclcpp::QoS(1).reliable().transient_local();
    plan_sub_ = create_subscription<nav_msgs::msg::Path>(
        "plan", plan_qos, [this](nav_msgs::msg::Path::ConstSharedPtr msg) { onPlan(*msg); });
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "odom", rclcpp::SensorDataQoS(),
        [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) { onOdom(*msg); });

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    cte_pub_ = create_publisher<std_msgs::msg::Float64>("~/cross_track_error", 10);

    timer_ = create_wall_timer(std::chrono::duration<double>(dt_), [this] { onTimer(); });

    RCLCPP_INFO(get_logger(), "controller '%s' running at %.1f Hz", controller_->name().c_str(),
                rate_hz);
  }

 private:
  std::unique_ptr<pnc_core::ControllerInterface> makeController(const std::string& type) {
    if (type == "pure_pursuit") {
      pnc_core::PurePursuitParams p;
      p.lookahead_gain = declare_parameter("pure_pursuit.lookahead_gain", p.lookahead_gain);
      p.lookahead_min = declare_parameter("pure_pursuit.lookahead_min", p.lookahead_min);
      p.lookahead_max = declare_parameter("pure_pursuit.lookahead_max", p.lookahead_max);
      p.speed_kp = declare_parameter("pure_pursuit.speed_kp", p.speed_kp);
      return std::make_unique<pnc_core::PurePursuit>(vehicle_, p);
    }
    // Phase 1 adds "stanley" and "mpc" here.
    throw std::invalid_argument("unknown controller.type: " + type);
  }

  void onPlan(const nav_msgs::msg::Path& msg) {
    // A latched (transient_local) plan can be delivered more than once during
    // discovery. Re-applying it would reset progress, so drop exact repeats.
    const rclcpp::Time stamp(msg.header.stamp, RCL_ROS_TIME);
    if (have_plan_ && stamp == last_plan_stamp_ && msg.poses.size() == path_.size()) {
      return;
    }
    have_plan_ = true;
    last_plan_stamp_ = stamp;

    pnc_core::Path path;
    path.reserve(msg.poses.size());
    for (const auto& ps : msg.poses) {
      path.push_back({ps.pose.position.x, ps.pose.position.y, 0.0, reference_speed_});
    }
    pnc_core::computeHeadings(path);
    if (!path.empty()) {
      path.back().v_ref = 0.0;  // stop at the end of the plan
    }
    path_ = std::move(path);
    controller_->reset();
    progress_idx_ = 0;
    goal_reached_ = false;
    RCLCPP_INFO(get_logger(), "received plan with %zu poses (frame '%s')", path_.size(),
                msg.header.frame_id.c_str());
  }

  void onOdom(const nav_msgs::msg::Odometry& msg) {
    const auto& p = msg.pose.pose;
    pnc_core::State s;
    s.x = p.position.x;
    s.y = p.position.y;
    s.yaw = yawFromQuaternion(p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w);
    s.v = msg.twist.twist.linear.x;
    state_ = s;
    last_odom_time_ = now();
  }

  void publishStop() {
    v_cmd_ = 0.0;
    cmd_pub_->publish(geometry_msgs::msg::Twist{});
  }

  void onTimer() {
    if (!state_ || path_.empty() || goal_reached_) {
      publishStop();
      return;
    }
    if ((now() - last_odom_time_).seconds() > odom_timeout_s_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "odometry stale, stopping");
      publishStop();
      return;
    }

    // Track progress along the path with a forward-only windowed search. The
    // goal check needs it: on a closed path the last waypoint coincides with
    // the first, so distance to the goal alone would stop the car at the start.
    progress_idx_ = pnc_core::nearestIndex(path_, state_->x, state_->y, progress_idx_, 50);
    const bool near_end = progress_idx_ + 10 >= path_.size();
    const auto& goal = path_.back();
    if (near_end && std::hypot(goal.x - state_->x, goal.y - state_->y) < goal_tolerance_) {
      goal_reached_ = true;
      RCLCPP_INFO(get_logger(), "goal reached");
      publishStop();
      return;
    }

    const pnc_core::ControlCommand cmd = controller_->computeCommand(*state_, path_, dt_);

    // The Gazebo Ackermann plugin takes (v, omega). Integrate the acceleration
    // command into the speed command and convert steering angle to yaw rate.
    // Integrating our own previous command (not the measured speed) makes the
    // speed loop an integrator, so a drive that delivers slightly less than
    // commanded (wheel slip, odometry filtering) still reaches v_ref.
    v_cmd_ = pnc_core::clamp(v_cmd_ + cmd.accel * dt_, 0.0, vehicle_.max_speed);
    geometry_msgs::msg::Twist twist;
    twist.linear.x = v_cmd_;
    twist.angular.z = model_->yawRate(v_cmd_, cmd.steer);
    cmd_pub_->publish(twist);

    std_msgs::msg::Float64 cte;
    cte.data = pnc_core::crossTrackError(path_, state_->x, state_->y);
    cte_pub_->publish(cte);
  }

  pnc_core::VehicleParams vehicle_;
  std::unique_ptr<pnc_core::ControllerInterface> controller_;
  std::unique_ptr<pnc_core::KinematicBicycle> model_;
  pnc_core::Path path_;
  std::optional<pnc_core::State> state_;
  rclcpp::Time last_odom_time_{0, 0, RCL_ROS_TIME};
  double dt_{0.02};
  double v_cmd_{0.0};  // speed command sent last cycle [m/s]
  double reference_speed_{1.0};
  double odom_timeout_s_{0.5};
  double goal_tolerance_{0.2};
  std::size_t progress_idx_{0};
  bool goal_reached_{false};
  bool have_plan_{false};
  rclcpp::Time last_plan_stamp_{0, 0, RCL_ROS_TIME};

  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr plan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr cte_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace pnc_control

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(pnc_control::TrackingControllerNode)
