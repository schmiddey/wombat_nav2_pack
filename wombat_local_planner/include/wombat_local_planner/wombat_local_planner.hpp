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

namespace wombat_local_planner{


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
   * @param robot_pose 
   * @return nav_msgs::msg::Path 
   */
  nav_msgs::msg::Path prepareGlobalPath(const geometry_msgs::msg::PoseStamped& robot_pose) const;

protected: 
  rclcpp_lifecycle::LifecycleNode::WeakPtr       _node;
  std::shared_ptr<tf2_ros::Buffer>               _tf;
  std::string                                    _plugin_name;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> _costmap_ros;
  
  rclcpp::Logger _logger {rclcpp::get_logger("WombatLocalPlanner")};

  rclcpp::Clock::SharedPtr _clock;

  //-- Params --
  // rclcpp::Duration _tf_tolerance;
  rclcpp::Duration _tf_tolerance{0, 0};
  double           _prune_distance = 99999.99;
  





  nav_msgs::msg::Path _global_plan;
  nav_msgs::msg::Path _transformed_global_plan;
  // -- -- 
  std::unique_ptr<wombat::LifecylePubHandler> _pub;
};

} //namespace wombat_local_planner

#endif  //WOMBAT_LOCAL_PLANNER_H_
