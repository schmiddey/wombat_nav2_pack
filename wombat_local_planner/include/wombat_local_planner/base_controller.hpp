#ifndef BASE_CONTROLLER_H_
#define BASE_CONTROLLER_H_

#include <iostream>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

#include "wombat_utility/wombat_types.hpp"
#include "wombat_utility/wombatLocalPath2.hpp"

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
   * @return geometry_msgs::msg::Twist  scales -1 .. 1
   */
  virtual geometry_msgs::msg::Twist control(const Pose2& robot_pose,
                                            const std::shared_ptr<LocalPath2>& local_path,
                                            const double end_approach_scale,
                                            const Pose2& tolerance = Pose2()) = 0;

protected:
  BaseController() = default;
  rclcpp::Logger _logger {rclcpp::get_logger("wombatLocalPlanner->Controller")};
};


} //namespace wombat

#endif  //BASE_CONTROLLER_H_