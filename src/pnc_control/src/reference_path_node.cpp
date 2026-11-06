// Copyright (c) 2026 Hu Zhiguo. MIT License.
//
// Publishes a fixed reference path on `plan` (latched). Stands in for the
// global planner until Hybrid A* is implemented, so the controller can be
// tested end to end in simulation from day one.

#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

#include "nav_msgs/msg/path.hpp"
#include "pnc_core/math_utils.hpp"
#include "pnc_core/path_utils.hpp"
#include "rclcpp/rclcpp.hpp"

namespace pnc_control {

class ReferencePathNode : public rclcpp::Node {
 public:
  explicit ReferencePathNode(const rclcpp::NodeOptions& options) : Node("reference_path", options) {
    const std::string shape = declare_parameter<std::string>("shape", "circle");
    const std::string frame = declare_parameter<std::string>("frame_id", "odom");
    const double size = declare_parameter<double>("size", 3.0);  // radius or length [m]
    const int n = declare_parameter<int>("num_points", 400);

    pnc_core::Path path;
    if (shape == "circle") {
      // Counter-clockwise circle centred at (0, size): it starts at the origin
      // heading +x, which is where the simulated vehicle spawns.
      for (int i = 0; i < n; ++i) {
        const double th = -pnc_core::kPi / 2.0 + 2.0 * pnc_core::kPi * i / n;
        path.push_back({size * std::cos(th), size + size * std::sin(th), 0.0, 0.0});
      }
      pnc_core::computeHeadings(path);
    } else if (shape == "straight") {
      path = pnc_core::makeStraightPath(0.0, 0.0, 0.0, size / n, static_cast<std::size_t>(n), 0.0);
    } else {
      throw std::invalid_argument("unknown shape: " + shape);
    }

    nav_msgs::msg::Path msg;
    msg.header.frame_id = frame;
    msg.header.stamp = now();
    for (const auto& w : path) {
      geometry_msgs::msg::PoseStamped ps;
      ps.header = msg.header;
      ps.pose.position.x = w.x;
      ps.pose.position.y = w.y;
      ps.pose.orientation.z = std::sin(w.yaw / 2.0);
      ps.pose.orientation.w = std::cos(w.yaw / 2.0);
      msg.poses.push_back(ps);
    }

    pub_ =
        create_publisher<nav_msgs::msg::Path>("plan", rclcpp::QoS(1).reliable().transient_local());
    pub_->publish(msg);
    RCLCPP_INFO(get_logger(), "published %s path with %zu poses in '%s'", shape.c_str(),
                msg.poses.size(), frame.c_str());
  }

 private:
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_;
};

}  // namespace pnc_control

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(pnc_control::ReferencePathNode)
