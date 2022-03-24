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
  _logger      = rclcpp::get_logger("wombatLocalPlanner");//node->get_logger();
  _clock       = node->get_clock();


  if(!_pub)
  {
    _pub = std::make_unique<wombat::LifecylePubHandler>(parent, name);
  }

  RCLCPP_INFO(_logger, "--------------------------------------------------");
  RCLCPP_INFO(_logger, "-- Wombat Local Planner -> Configure... Params: --");
  RCLCPP_INFO(_logger, "--------------------------------------------------");

  nav2_util::declare_parameter_if_not_declared(node, _plugin_name + ".transform_tolerance", rclcpp::ParameterValue(2.0));
  nav2_util::declare_parameter_if_not_declared(node, _plugin_name + ".local_target_dist",   rclcpp::ParameterValue(0.2));
  nav2_util::declare_parameter_if_not_declared(node, _plugin_name + ".local_path_max_dist", rclcpp::ParameterValue(50.0));
  nav2_util::declare_parameter_if_not_declared(node, _plugin_name + ".end_approach_dist",   rclcpp::ParameterValue(1.0));
  // nav2_util::declare_parameter_if_not_declared(node, _plugin_name + ".transform_tolerance", rclcpp::ParameterValue(0.1));

  node->get_parameter(_plugin_name + ".local_target_dist",    _local_target_dist);
  node->get_parameter(_plugin_name + ".local_path_max_dist",  _local_path_max_dist);
  node->get_parameter(_plugin_name + ".end_approach_dist",    _end_approach_dist);

  // node->get_parameter(_plugin_name + ".end_approach_dist",    _end_approach_dist);
  
  double transform_tolerance;
  node->get_parameter(_plugin_name + ".transform_tolerance", transform_tolerance);
  _tf_tolerance = rclcpp::Duration::from_seconds(transform_tolerance);


  RCLCPP_INFO(_logger, "transform_tolerance: %f s", transform_tolerance);
  RCLCPP_INFO(_logger, "local_target_dist: %f m", _local_target_dist);
  RCLCPP_INFO(_logger, "local_path_max_dist: %f m", _local_path_max_dist);
  RCLCPP_INFO(_logger, "end_approach_dist: %f m", _end_approach_dist);

  //todo other controller via plugin!!  
  _controller = std::make_unique<wombat::MecanumController>();
  _controller->initialize(parent, name);

  _pub->on_configure();

  RCLCPP_INFO(_logger, "--------------------------------------------------");
  RCLCPP_INFO(_logger, "--------------------------------------------------");
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
  //only for compile stuff
  geometry_msgs::msg::TwistStamped msg;
  msg.twist = velocity;


  RCLCPP_INFO(_logger, "pose_in frame: %s", pose.header.frame_id.c_str());

  auto goal_is_reached = goal_checker->isGoalReached(pose.pose, _global_path.poses.back().pose, velocity);
  if(goal_is_reached)
  {
    RCLCPP_INFO(_logger, "Goal is Reached");

  }


  //todo if last path element is shorter than 
  _local_path_unmodified = this->prepareGlobalPath(pose, _global_path);
  _pub->publish_local_plan(_local_path_unmodified);

  //tmp fix for cropped path
  auto path_length = wombat::Utility::computePathLength(_local_path_unmodified);
  auto dist_to_goal = wombat::Utility::computeSquareDistance2D(pose.pose, _local_path_unmodified.poses.back().pose );

  auto final_length = std::max(path_length, dist_to_goal);
  double end_approach_scale = 1.0;
  if(final_length < _end_approach_dist)
  {
    end_approach_scale = wombat::Utility::rescale(final_length, 0.0, _end_approach_dist, 0.0, 1.0);
  }

  // RCLCPP_INFO(_logger, "end_approach_scale(0.0..1.0): %f", end_approach_scale);

  msg = _controller->control(pose, _local_path_unmodified, end_approach_scale);
  msg.header.stamp = pose.header.stamp;
  msg.header.frame_id = "todo";


  return msg;
}

void WombatLocalPlanner::setPlan(const nav_msgs::msg::Path& path) 
{
  _pub->publish_global_plan(path);
  _global_path = path;
  RCLCPP_INFO(_logger, "Got new Path... -- frame: %s  --  size: %d, length: %f m:", path.header.frame_id.c_str(), static_cast<int>(path.poses.size()), wombat::Utility::computePathLength(path));
}


void WombatLocalPlanner::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  RCLCPP_ERROR(_logger, "setSpeedLimit!!!!");

  double sl = speed_limit;
  bool   per = percentage;

  RCLCPP_ERROR(_logger, "speedlimit: %f , use percent %s", sl, (per ? "true" : "false"));
}

/**
 * @brief 
 * 
 * @todo check and ensure that first pose of ret_path is roughly(maybe configurable) at local_target_dist, if not interpolate
 * 
 * @todo REDO!!!! like in python script
 * 
 * @param robot_pose 
 * @param global_path 
 * @return nav_msgs::msg::Path 
 */
nav_msgs::msg::Path WombatLocalPlanner::prepareGlobalPath(const geometry_msgs::msg::PoseStamped& robot_pose, const nav_msgs::msg::Path& global_path) const
{
  //Do it as in dwb controller
  // std::cout << "--" << std::endl;
  // std::cout << " enter prepareGlobalPath" << std::endl;
  // std::cout << "cost_map_global_frame: " << _costmap_ros->getGlobalFrameID() << std::endl;
  // std::cout << "robot_pose_frame:      " << robot_pose.header.frame_id << std::endl;

  if(global_path.poses.empty())
  {
    throw nav2_core::PlannerException("Received empty global path");
  }

  //first transform the robot pose in global path frame , trim global path, then transform global path into robot_pose 
  //frame to prevent transforming unnecessary poses from global path
  
  auto t_robot_pose = wombat::TfUtility::transform(_tf, robot_pose, global_path.header.frame_id, _tf_tolerance);
  if(!t_robot_pose)
  {
    throw nav2_core::PlannerException("Unable to transform robot pose into global plan's frame");
  }
  //trim global path by the points outside the local costmap
  auto* costmap = _costmap_ros->getCostmap();

  double dist_threshold = std::max(costmap->getSizeInCellsX(), costmap->getSizeInCellsY()) * costmap->getResolution() / 2.0;
  
  //min dist for first path element
  double sq_path_min_dist = std::min(dist_threshold, _local_target_dist);
  // std::cout << "dist_threshold: " << dist_threshold << std::endl;
  // std::cout << "_local_target_dist: " << _local_target_dist << std::endl;
  // std::cout << "min ->  sq_path_min_dist: " << sq_path_min_dist << std::endl;
  sq_path_min_dist = sq_path_min_dist * sq_path_min_dist;


  //max dist for last path element
  double sq_path_max_dist = std::min(dist_threshold, _local_path_max_dist);
  sq_path_max_dist = sq_path_max_dist * sq_path_max_dist;

  
  //find first pose thats less than sq_path_min_dist from the robot_pose
  auto transformation_rbegin = std::find_if(global_path.poses.rbegin(), global_path.poses.rend(),
                                           [&](const auto & global_plan_pose) {
                                             return wombat::Utility::computeSquareDistance2D(t_robot_pose.value().pose, global_plan_pose.pose) < sq_path_min_dist;
                                           }
                                           );

  //create iterator from reverse iterator:
  auto transformation_begin = transformation_rbegin.base() -1;
  if(transformation_rbegin == global_path.poses.rend()) //todo check if correct
  {
    transformation_begin = global_path.poses.begin();
  }
  //find fist pose thats further than sq_path_max_dist from robot_pose
  auto transformation_end = std::find_if(transformation_begin, global_path.poses.end(),
                                         [&](const auto & global_plan_pose) {
                                           return wombat::Utility::computeSquareDistance2D(t_robot_pose.value().pose, global_plan_pose.pose) > sq_path_max_dist;
                                         }
                                         );
  
  nav_msgs::msg::Path transformed_global_path;
  transformed_global_path.header = global_path.header;
  //create container with poses within the local range
  transformed_global_path.poses = decltype(transformed_global_path.poses)(transformation_begin, transformation_end);

  // RCLCPP_INFO(_logger, "size global path: %d", (int)global_path.poses.size());
  // RCLCPP_INFO(_logger, "idx begin: %d", (int)std::distance(global_path.poses.begin(), transformation_begin));
  // RCLCPP_INFO(_logger, "idx end  : %d", (int)std::distance(global_path.poses.begin(), transformation_end));


  // RCLCPP_INFO(_logger, "size of tranformed_path: %d", (int)transformed_global_path.poses.size());

  for(auto& e : transformed_global_path.poses)
  {
    e.header = transformed_global_path.header;
  }

  transformed_global_path = wombat::TfUtility::transform(_tf, transformed_global_path, _costmap_ros->getGlobalFrameID(), _tf_tolerance).value_or(nav_msgs::msg::Path());

  RCLCPP_INFO(_logger, "_costmap_ros->getGlobalFrameID(): %s", _costmap_ros->getGlobalFrameID().c_str());

  if (transformed_global_path.poses.empty()) {
    throw nav2_core::PlannerException("Resulting plan has 0 poses in it.");
  }

  transformed_global_path.header.stamp = robot_pose.header.stamp;

  // std::cout << "  leave prepareGlobalPath" << std::endl;
  // std::cout << "--" << std::endl;

  return transformed_global_path;
}



} // namespace wombat_local_planner

// Register this controller as a nav2_core plugin
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(wombat_local_planner::WombatLocalPlanner, nav2_core::Controller)


