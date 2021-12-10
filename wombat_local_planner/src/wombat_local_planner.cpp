#include "wombat_local_planner/wombat_local_planner.hpp"




namespace wombat_local_planner
{

void WombatLocalPlanner::configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent,
                                   std::string name, 
                                   const std::shared_ptr<tf2_ros::Buffer>& tf,
                                   const std::shared_ptr<nav2_costmap_2d::Costmap2DROS>& costmap_ros  
                                   )
{
  _node = parent;

  auto node = _node.lock();

  _costmap_ros = costmap_ros;
  _tf          = tf;
  _plugin_name = name;
  _logger      = node->get_logger();
  _clock       = node->get_clock();


  if(!_pub)
  {
    _pub = std::make_unique<wombat::LifecylePubHandler>(parent, name);
  }

  // nav2_util::declare_parameter_if_not_declared(node, _plugin_name + ".desired_linear_vel", rclcpp::ParameterValue(0.2));
  // nav2_util::declare_parameter_if_not_declared(node, _plugin_name + ".lookahead_dist", rclcpp::ParameterValue(0.4));
  // nav2_util::declare_parameter_if_not_declared(node, _plugin_name + ".max_angular_vel", rclcpp::ParameterValue(1.0));
  // nav2_util::declare_parameter_if_not_declared(node, _plugin_name + ".end_approach_dist", rclcpp::ParameterValue(1.0));
  // nav2_util::declare_parameter_if_not_declared(node, _plugin_name + ".transform_tolerance", rclcpp::ParameterValue(0.1));

  // node->get_parameter(_plugin_name + ".desired_linear_vel", _desired_linear_vel);
  // node->get_parameter(_plugin_name + ".lookahead_dist",     _lookahead_dist);
  // node->get_parameter(_plugin_name + ".max_angular_vel",    _max_angular_vel);
  // node->get_parameter(_plugin_name + ".end_approach_dist",    _end_approach_dist);
  
  double transform_tolerance;
  node->get_parameter(_plugin_name + ".transform_tolerance", transform_tolerance);
  _tf_tolerance = rclcpp::Duration::from_seconds(transform_tolerance);


  _pub->on_configure();
}

void WombatLocalPlanner::cleanup() 
{
  RCLCPP_INFO(_logger, "Cleaning up controller: %s", _plugin_name.c_str());
  _pub->on_cleanup();
}

void WombatLocalPlanner::activate() 
{
  RCLCPP_INFO(_logger, "Activating controller: %s",    _plugin_name.c_str());
  _pub->on_activate();
}

void WombatLocalPlanner::deactivate() 
{
  RCLCPP_INFO(_logger, "Dectivating controller: %s", _plugin_name.c_str());
  _pub->on_deactivate();
}

geometry_msgs::msg::TwistStamped WombatLocalPlanner::computeVelocityCommands(const geometry_msgs::msg::PoseStamped& pose,
                                                                             const geometry_msgs::msg::Twist& velocity, 
                                                                             nav2_core::GoalChecker* goal_checker) 
{
  RCLCPP_INFO(_logger, "pose_in frame: %s", pose.header.frame_id.c_str());

  geometry_msgs::msg::TwistStamped msg;
  msg.twist = velocity;


  goal_checker->isGoalReached(pose.pose, pose.pose, velocity);
  
  return msg;
}

void WombatLocalPlanner::setPlan(const nav_msgs::msg::Path& path) 
{
  _pub->publish_global_plan(path);
  // Transform global path into the robot's frame

  // _global_plan = transformGlobalPlan(path);
}


void WombatLocalPlanner::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  RCLCPP_ERROR(_logger, "setSpeedLimit!!!!");

  double sl = speed_limit;
  bool   per = percentage;

  RCLCPP_ERROR(_logger, "speedlimit: %f , use percent %s", sl, (per ? "true" : "false"));
}


nav_msgs::msg::Path WombatLocalPlanner::prepareGlobalPath(const geometry_msgs::msg::PoseStamped& robot_pose) const
{
  //Do it as in dwb controller

  if(_global_plan.poses.empty())
  {
    throw nav2_core::PlannerException("Received empty global path");
  }

  //first transform the robot pose in global path frame , trim global path, then transform global path into robot_pose 
  //frame to prevent transforming unnecessary poses from global path
  
  auto t_robot_pose = wombat::TfUtility::transform(_tf, robot_pose, _global_plan.header.frame_id, _tf_tolerance);
  if(!t_robot_pose)
  {
    throw nav2_core::PlannerException("Unable to transform robot pose into global plan's frame");
  }
  
  //trim global path by the points outside the local costmap
  auto* costmap = _costmap_ros->getCostmap();
  double dist_threshold = std::max(costmap->getSizeInCellsX(), costmap->getSizeInCellsY()) * costmap->getResolution() / 2.0;
  double sq_dist_threshold = dist_threshold * dist_threshold;

  

  nav_msgs::msg::Path ret;

  return ret;
}



} // namespace wombat_local_planner

// Register this controller as a nav2_core plugin
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(wombat_local_planner::WombatLocalPlanner, nav2_core::Controller)


