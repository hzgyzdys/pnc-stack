// Copyright (c) 2026 Hu Zhiguo. MIT License.
//
// Publishes a fixed reference path on `plan` (latched). Stands in for the
// global planner until Hybrid A* is implemented, so the controller can be
// tested end to end in simulation from day one.

#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

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
    } else if (shape == "segments") {
      // Lines and arcs chained from a start pose, e.g. for the test scenarios
      // in pnc_bringup/config/scenarios.
      const double x0 = declare_parameter<double>("start_x", 0.0);
      const double y0 = declare_parameter<double>("start_y", 0.0);
      const double yaw0 = declare_parameter<double>("start_yaw", 0.0);
      const double ds = declare_parameter<double>("resolution", 0.05);
      const auto texts =
          declare_parameter<std::vector<std::string>>("segments", std::vector<std::string>{});
      std::vector<pnc_core::PathSegment> segments;
      for (const auto& t : texts) {
        const auto seg = pnc_core::parseSegment(t);
        if (!seg) {
          throw std::invalid_argument("bad segment '" + t +
                                      "', expected 'line <len>' or 'arc <radius> <deg>'");
        }
        segments.push_back(*seg);
      }
      if (segments.empty()) {
        throw std::invalid_argument("shape 'segments' needs a non-empty 'segments' list");
      }
      path = pnc_core::makeSegmentPath(x0, y0, yaw0, segments, ds, 0.0);
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
