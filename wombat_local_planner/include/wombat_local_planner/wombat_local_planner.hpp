#ifndef WOMBAT_LOCAL_PLANNER_H_
#define WOMBAT_LOCAL_PLANNER_H_


#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "nav2_core/controller.hpp"
#include "nav2_core/exceptions.hpp"
#include "nav2_util/node_utils.hpp"

#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"

#include "lifecycle_publisher.hpp"

#include "wombat_utility/tf_utility.hpp"
#include "wombat_utility/utility.hpp"
#include "wombat_utility/wombatPath2.hpp"
#include "wombat_utility/wombatLocalPath2.hpp"

#include "base_controller.hpp"
#include "mecanum_controller.hpp"

namespace wombat_local_planner{

/**
 * @brief 
 * 
 * @todo use pluginlinb for :  controller (Mecanum, Differential)( speed depends on curvage? also as plugin?)
 *                             CommandVelocity critics (Max Accel, Max speed)
 *                             more!!!?
 *                             path_decorator/Orientation planner (Mecanum only) -> planning orientation of path
 * 
 */
class WombatLocalPlanner: public nav2_core::Controller {
public:
  WombatLocalPlanner() = default;
  ~WombatLocalPlanner() override = default;

  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent,
                 std::string name, 
                 const std::shared_ptr<tf2_ros::Buffer>& tf,
                 const std::shared_ptr<nav2_costmap_2d::Costmap2DROS>& costmap_ros  
                ) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;
  
  // geometry_msgs::msg::TwistStamped computeVelocityCommands(const geometry_msgs::msg::PoseStamped & pose,
  //                                                          const geometry_msgs::msg::Twist & velocity) override;
  
  geometry_msgs::msg::TwistStamped computeVelocityCommands(const geometry_msgs::msg::PoseStamped& pose,
                                                           const geometry_msgs::msg::Twist& velocity, 
                                                           nav2_core::GoalChecker* goal_checker) override;

  void setPlan(const nav_msgs::msg::Path& path) override;

  void setSpeedLimit(const double& speed_limit, const bool& percentage) override;

  
protected:

  /**
   * @brief prepare / trim / transform global path as in dwb controller
   * 
   * @todo if target pose from path is closer than local target dist return final pose
   * 
   * @param robot_pose 
   * @return nav_msgs::msg::Path 
   */
  nav_msgs::msg::Path prepareGlobalPath(const geometry_msgs::msg::PoseStamped& robot_pose, const nav_msgs::msg::Path& global_path) const;

protected: 
  rclcpp_lifecycle::LifecycleNode::WeakPtr       _node;
  std::shared_ptr<tf2_ros::Buffer>               _tf;
  std::string                                    _plugin_name;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> _costmap_ros;
  
  rclcpp::Logger _logger {rclcpp::get_logger("WombatLocalPlanner")};

  rclcpp::Clock::SharedPtr _clock;


  std::unique_ptr<wombat::BaseController> _controller;

  //-- Params --
  // rclcpp::Duration _tf_tolerance;
  rclcpp::Duration _tf_tolerance{0, 0};
  double           _local_target_dist   = 0.2;  //dist a path element is defined as reached
  double           _local_path_max_dist = 50.0;
  double           _end_approach_dist   = 1.0;

  // double           _final_target_dist = 0.05; //todo check if needed
  




  
  nav_msgs::msg::Path _global_path;
  nav_msgs::msg::Path _local_path_unmodified;
  // nav_msgs::msg::Path _transformed_global_plan;
  // -- -- 
  std::unique_ptr<wombat::LifecylePubHandler> _pub;
};

} //namespace wombat_local_planner

#endif  //WOMBAT_LOCAL_PLANNER_H_
