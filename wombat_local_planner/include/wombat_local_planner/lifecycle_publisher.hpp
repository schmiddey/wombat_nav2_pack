#ifndef LIFECYCLE_PUBLISHER_H_
#define LIFECYCLE_PUBLISHER_H_

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>

#include <nav2_util/lifecycle_node.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav2_util/node_utils.hpp>
// #include "nav2_costmap_2d/costmap_2d_ros.hpp"
// #include "visualization_msgs/msg/marker_array.hpp"


namespace wombat
{

class LifecylePubHandler{
public:
  explicit LifecylePubHandler(const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent, const std::string& plugin_name);
  // ~LifecylePubHandler();

  nav2_util::CallbackReturn on_configure();
  nav2_util::CallbackReturn on_activate();
  nav2_util::CallbackReturn on_deactivate();
  nav2_util::CallbackReturn on_cleanup();

  void publish_global_plan(const nav_msgs::msg::Path& path);
  void publish_local_plan(const nav_msgs::msg::Path& path);
  void publish_local_plan_unmodified(const nav_msgs::msg::Path& path);
  void publish_footprint_em_stop(const geometry_msgs::msg::PolygonStamped& footprint);
  void publish_footprint_safety(const geometry_msgs::msg::PolygonStamped& footprint);
  void publish_dbg_string(const std::string& msg);
  
private:
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>> _pub_global_path;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>> _pub_local_path;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>> _pub_local_path_unmodified;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PolygonStamped>> _pub_footprint_em_stop;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PolygonStamped>> _pub_footprint_safety;
  
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>> _pub_dbg_string;


  rclcpp_lifecycle::LifecycleNode::WeakPtr _node;
  rclcpp::Clock::SharedPtr _clock;
  std::string _plugin_name;
};

} // namespace wombat




#endif  //LIFECYCLE_PUBLISHER_H_