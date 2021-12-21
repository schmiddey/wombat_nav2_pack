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

  virtual void initialize() = 0;
  
  /**
   * @brief first pose of the local path must have at least the defined distance
   * 
   * @param robot_pose 
   * @return geometry_msgs::msg::TwistStamped 
   */
  virtual geometry_msgs::msg::TwistStamped control(const geometry_msgs::msg::PoseStamped& robot_pose, const nav_msgs::msg::Path& local_path) = 0;

protected:
  BaseController() = default;
};


} //namespace wombat

#endif  //BASE_CONTROLLER_H_