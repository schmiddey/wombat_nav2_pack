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
  // RCLCPP_INFO(_logger, "compute vel cmd");

  geometry_msgs::msg::TwistStamped msg;

  // RCLCPP_INFO(_logger, "pose_in frame: %s", pose.header.frame_id.c_str());
  auto robot_pose = wombat::Pose2(pose.pose);

  
  //todo Error ... poses to compare are not in the same frame :(  fix this
  auto goal_is_reached = goal_checker->isGoalReached(pose.pose, _global_path_unmodified.poses.back().pose, velocity);
  if(goal_is_reached)
  {
    RCLCPP_INFO(_logger, "Goal is Reached");

  }

  //if path is empty than send zero velocity
  if(_local_path->empty())
  {
    msg.header = pose.header;
    msg.header.frame_id = "todo";
    msg.twist.linear.x = 0.0;
    msg.twist.linear.y = 0.0;
    msg.twist.linear.z = 0.0;
    msg.twist.angular.x = 0.0;
    msg.twist.angular.y = 0.0;
    msg.twist.angular.z = 0.0;
    return msg;
  }
  

  _local_path->update(robot_pose);


  //publish local path
  auto tmp_local_path = _local_path->extractLocalPath();
  //prove if tmp_local path is empty
  if(tmp_local_path.empty())
  {
    RCLCPP_INFO(_logger, "tmp_local_path is empty");
    msg.header = pose.header;
    msg.header.frame_id = "todo";
    msg.twist.linear.x = 0.0;
    msg.twist.linear.y = 0.0;
    msg.twist.linear.z = 0.0;
    msg.twist.angular.x = 0.0;
    msg.twist.angular.y = 0.0;
    msg.twist.angular.z = 0.0;
    return msg;
  }

  // std::cout << "--" << std::endl;
  // std::cout << "size_tmp_local_path:         " << tmp_local_path.size() << std::endl;
  // std::cout << "final target:                " << _local_path->getFinalTarget() << std::endl;
  // std::cout << "last target from local_path: " << tmp_local_path.poses().back() << std::endl;
  // std::cout << "--" << std::endl;
  _pub->publish_local_plan(tmp_local_path.toRosPath());

  // //tmp fix for cropped path
  auto path_length = _local_path->localPathLength_from_begin();
  auto dist_to_goal = (robot_pose.position - _local_path->getFinalTarget().position).norm();

  auto final_length = std::max(path_length, dist_to_goal);
  double end_approach_scale = 1.0;
  if(final_length < _end_approach_dist)
  {
    end_approach_scale = wombat::Utility::rescale(final_length, 0.0, _end_approach_dist, 0.0, 1.0);
  }

  // RCLCPP_INFO(_logger, "end_approach_scale(0.0..1.0): %f", end_approach_scale);
  // std::cout << "end_approach_scale: " << end_approach_scale << std::endl;

  msg.twist = _controller->control(robot_pose, _local_path, end_approach_scale);



  msg.header.stamp = pose.header.stamp;
  msg.header.frame_id = "todo";


  return msg;
}

void WombatLocalPlanner::setPlan(const nav_msgs::msg::Path& path) 
{
  _pub->publish_global_plan(path);
  _global_path_unmodified = path;

  //transform global path into robot_frame
  auto transformed_global_path = wombat::TfUtility::transform(_tf, path, _costmap_ros->getGlobalFrameID(), _tf_tolerance).value_or(nav_msgs::msg::Path());
  if(transformed_global_path.poses.empty())
  {
    RCLCPP_ERROR(_logger, "transformed_global_path is empty.... maybe unable to be transformed");
  }

  auto* costmap = _costmap_ros->getCostmap();

  double dist_threshold = std::max(costmap->getSizeInCellsX(), costmap->getSizeInCellsY()) * costmap->getResolution() / 2.0;
  

  auto path2 = wombat::Path2(transformed_global_path);

  //create local path obj
  _local_path = std::make_shared<wombat::LocalPath2>(path2,
                                                     std::min(dist_threshold, _local_target_dist), 
                                                     std::min(dist_threshold, _local_path_max_dist) );

  RCLCPP_INFO(_logger, "Got new Path... -- frame: %s  --  size: %d, length: %f m:", path.header.frame_id.c_str(), static_cast<int>(path.poses.size()), wombat::Utility::computePathLength(path));
}


void WombatLocalPlanner::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  RCLCPP_ERROR(_logger, "setSpeedLimit!!!!");

  double sl = speed_limit;
  bool   per = percentage;

  RCLCPP_ERROR(_logger, "speedlimit: %f , use percent %s", sl, (per ? "true" : "false"));
}


} // namespace wombat_local_planner

// Register this controller as a nav2_core plugin
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(wombat_local_planner::WombatLocalPlanner, nav2_core::Controller)


