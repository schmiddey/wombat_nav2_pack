#ifndef BASE_CONTROLLER_H_
#define BASE_CONTROLLER_H_

#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

namespace wombat{

/**
 * @brief 
 * @todo use pluginlib later
 */
class BaseController{
public:
  virtual ~BaseController() = default;

  virtual void initialize(const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent,
                          const std::string& name) = 0;
  
  /**
   * @brief 
   * 
   * @param robot_pose         -> current robot pose
   * @param local_path         -> current local path, first pose of the local path must have at least the defined distance, unless its the final target pose (todo)
   * @param end_approach_scale -> scale between 1..0 how far the robot is from the final goal
   * 
   * @return geometry_msgs::msg::TwistStamped  scales -1 .. 1
   */
  virtual geometry_msgs::msg::TwistStamped control(const geometry_msgs::msg::PoseStamped& robot_pose,
                                                   const nav_msgs::msg::Path& local_path,
                                                   const double end_approach_scale) = 0;

protected:
  BaseController() = default;
  rclcpp::Logger _logger {rclcpp::get_logger("wombatLocalPlanner->Controller")};
};


} //namespace wombat

#endif  //BASE_CONTROLLER_H_