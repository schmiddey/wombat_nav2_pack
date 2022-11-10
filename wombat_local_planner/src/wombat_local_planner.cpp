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
  _costmap     = std::make_shared<wombat::Costmap2Extend>(costmap_ros->getCostmap());
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

  nav2_util::declare_parameter_if_not_declared(node, _plugin_name + ".transform_tolerance",     rclcpp::ParameterValue(2.0));
  nav2_util::declare_parameter_if_not_declared(node, _plugin_name + ".local_target_dist",       rclcpp::ParameterValue(0.2));
  nav2_util::declare_parameter_if_not_declared(node, _plugin_name + ".local_path_max_dist",     rclcpp::ParameterValue(50.0));
  nav2_util::declare_parameter_if_not_declared(node, _plugin_name + ".end_approach_dist",       rclcpp::ParameterValue(1.0));
  nav2_util::declare_parameter_if_not_declared(node, _plugin_name + ".max_accel_lin",           rclcpp::ParameterValue(1.0));
  nav2_util::declare_parameter_if_not_declared(node, _plugin_name + ".max_accel_ang",           rclcpp::ParameterValue(1.0));
  nav2_util::declare_parameter_if_not_declared(node, _plugin_name + ".footprint_em_stop_scale", rclcpp::ParameterValue(1.2));
  nav2_util::declare_parameter_if_not_declared(node, _plugin_name + ".footprint_safety_scale",  rclcpp::ParameterValue(1.5));
  nav2_util::declare_parameter_if_not_declared(node, _plugin_name + ".em_stop_zone_cell_cnt",   rclcpp::ParameterValue(1));
  nav2_util::declare_parameter_if_not_declared(node, _plugin_name + ".safety_zone_cell_cnt",    rclcpp::ParameterValue(1));
  nav2_util::declare_parameter_if_not_declared(node, _plugin_name + ".safety_zone_speed_scale", rclcpp::ParameterValue(0.5));


  node->get_parameter(_plugin_name + ".local_target_dist",    _local_target_dist);
  node->get_parameter(_plugin_name + ".local_path_max_dist",  _local_path_max_dist);
  node->get_parameter(_plugin_name + ".end_approach_dist",    _end_approach_dist);

  double max_accel_lin;
  double max_accel_ang;

  node->get_parameter(_plugin_name + ".max_accel_lin", max_accel_lin);
  node->get_parameter(_plugin_name + ".max_accel_ang", max_accel_ang);

  // node->get_parameter(_plugin_name + ".end_approach_dist",    _end_approach_dist);
  
  double transform_tolerance;
  node->get_parameter(_plugin_name + ".transform_tolerance", transform_tolerance);
  _tf_tolerance = rclcpp::Duration::from_seconds(transform_tolerance);

  double footprint_em_stop_scale;
  node->get_parameter(_plugin_name + ".footprint_em_stop_scale", footprint_em_stop_scale);

  double footprint_safety_scale;
  node->get_parameter(_plugin_name + ".footprint_safety_scale", footprint_safety_scale);


  _params.robot_footprint = wombat::Polygon2d(costmap_ros->getRobotFootprint());
  _params.robot_radius    = costmap_ros->getRobotRadius();
  _params.robot_footprint_em_stop = _params.robot_footprint.scaled(footprint_em_stop_scale);
  _params.robot_footprint_safety = _params.robot_footprint.scaled(footprint_safety_scale);  

  node->get_parameter(_plugin_name + ".em_stop_zone_cell_cnt", _params.em_stop_zone_cell_cnt);
  node->get_parameter(_plugin_name + ".safety_zone_cell_cnt",  _params.safety_zone_cell_cnt);
  _params.em_stop_zone_cell_cnt = std::abs(_params.em_stop_zone_cell_cnt);
  _params.safety_zone_cell_cnt  = std::abs(_params.safety_zone_cell_cnt);
  node->get_parameter(_plugin_name + ".safety_zone_speed_scale", _params.safety_zone_speed_scale);


  
  RCLCPP_INFO(_logger, "transform_tolerance: %f s", transform_tolerance);
  RCLCPP_INFO(_logger, "local_target_dist: %f m", _local_target_dist);
  RCLCPP_INFO(_logger, "local_path_max_dist: %f m", _local_path_max_dist);
  RCLCPP_INFO(_logger, "end_approach_dist: %f m", _end_approach_dist);
  RCLCPP_INFO(_logger, "max_accel_lin: %f m/s^2", max_accel_lin);
  RCLCPP_INFO(_logger, "max_accel_ang: %f rad/s^2", max_accel_ang);
  RCLCPP_INFO(_logger, "robot_radius: %f m", _params.robot_radius);
  RCLCPP_INFO_STREAM(_logger, "footprint: " << _params.robot_footprint);
  RCLCPP_INFO(_logger, "footprint_safety_scale: %f", footprint_safety_scale);
  RCLCPP_INFO(_logger, "em_stop_zone_cell_cnt: %d",_params.em_stop_zone_cell_cnt);
  RCLCPP_INFO(_logger, "safety_zone_cell_cnt:  %d",_params.safety_zone_cell_cnt);
  RCLCPP_INFO(_logger, "safety_zone_speed_scale: %f", _params.safety_zone_speed_scale);
  RCLCPP_INFO_STREAM(_logger, "footprint_safety: " << _params.robot_footprint_safety);
  RCLCPP_INFO_STREAM(_logger, "footprint_em_stop: " << _params.robot_footprint_em_stop);


  //todo other controller via plugin!!  
  _controller = std::make_unique<wombat::MecanumController>();
  _controller->initialize(parent, name);

  //limit accel
  _limit_accel = std::make_unique<wombat::LimitAccel>(_clock, max_accel_lin, max_accel_ang);

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

  //set header to footprints
  std_msgs::msg::Header header;
  header.frame_id = _costmap_ros->getBaseFrameID();
  header.stamp = _clock->now();

  _params.robot_footprint.header() = header;
  _params.robot_footprint_safety.header() = header;
  _params.robot_footprint_em_stop.header() = header;


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

  //pub footprint
  _pub->publish_footprint_em_stop(_params.robot_footprint_em_stop.toRosPolygon(_costmap_ros->getBaseFrameID(), pose.header.stamp));
  _pub->publish_footprint_safety(_params.robot_footprint_safety.toRosPolygon(_costmap_ros->getBaseFrameID(), pose.header.stamp));


  // check zones

  //compute current robot zone by transforming the footprint into the global frame
  _params.robot_footprint.header().stamp = pose.header.stamp;
  _params.robot_footprint_safety.header().stamp = pose.header.stamp;
  _params.robot_footprint_em_stop.header().stamp = pose.header.stamp;

  std::cout << "zone default: " << _params.robot_footprint_safety << std::endl;

  auto robot_zone_slow = wombat::TfUtility::transform(_tf, _params.robot_footprint_safety, _costmap_ros->getGlobalFrameID(), _tf_tolerance);
  if(!robot_zone_slow)
  {
    RCLCPP_ERROR(_logger, "Could not transform robot footprint into global frame");
    return wombat::Utility::getEmptyTwist(pose.header.stamp, _costmap_ros->getBaseFrameID());
  }

  auto robot_zone_em_stop = wombat::TfUtility::transform(_tf, _params.robot_footprint_em_stop, _costmap_ros->getGlobalFrameID(), _tf_tolerance);
  if(!robot_zone_em_stop)
  {
    RCLCPP_ERROR(_logger, "Could not transform robot footprint into global frame");
    return wombat::Utility::getEmptyTwist(pose.header.stamp, _costmap_ros->getBaseFrameID());
  }


  double zone_slow    = wombat::SafetyZoneChecker::check(robot_zone_slow.value(), _costmap);
  double zone_em_stop = wombat::SafetyZoneChecker::check(robot_zone_em_stop.value(), _costmap);

  //if path is empty than send zero velocity
  if(_local_path->empty())
  {
    return wombat::Utility::getEmptyTwist(pose.header.stamp, _costmap_ros->getBaseFrameID());
  }
  
  _local_path->update(robot_pose);

  //publish local path
  auto tmp_local_path = _local_path->extractLocalPath();
  //prove if tmp_local path is empty
  if(tmp_local_path.empty())
  {
    RCLCPP_INFO(_logger, "tmp_local_path is empty");

    return wombat::Utility::getEmptyTwist(pose.header.stamp, _costmap_ros->getBaseFrameID());
  }

  // std::cout << "--" << std::endl;
  // std::cout << "size_tmp_local_path:         " << tmp_local_path.size() << std::endl;
  // std::cout << "final target:                " << _local_path->getFinalTarget() << std::endl;
  // std::cout << "last target from local_path: " << tmp_local_path.poses().back() << std::endl;
  // std::cout << "--" << std::endl;
  _pub->publish_local_plan_unmodified(tmp_local_path.toRosPath());



  //modify path based on costmap and pub local path
  // auto* cm = _costmap_ros->getCostmap();
  // wombat::Costmap2DWombat cm(*(_costmap_ros->getCostmap())); //copy
  // auto hans = cm.getResolution();

  // if(hans > 0)
  // {
  //   RCLCPP_INFO(_logger, "hans: %f", hans);
  // }
  // else
  // {
  //   RCLCPP_INFO(_logger, "hans: %f", hans);
  // }

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
  //apply emstop if needed
  if(zone_em_stop > static_cast<double>(_params.em_stop_zone_cell_cnt))
  {
    RCLCPP_INFO(_logger, "zone_em_stop > 1.0");
    return wombat::Utility::getEmptyTwist(pose.header.stamp, _costmap_ros->getBaseFrameID());
  }

  auto tmp_twist = _controller->control(robot_pose, _local_path, end_approach_scale);

  //apply zone slow down
  double vel_fac = 1.0;
  if(zone_slow > static_cast<double>(_params.safety_zone_cell_cnt))
  {
    vel_fac = _params.safety_zone_speed_scale;
  }

  msg.twist = _limit_accel->limitAccel(tmp_twist, vel_fac);



  msg.header.stamp = pose.header.stamp;
  msg.header.frame_id = _costmap_ros->getBaseFrameID();


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

  double dist_threshold = std::max(_costmap->costmap().getSizeInCellsX(), _costmap->costmap().getSizeInCellsY()) * _costmap->costmap().getResolution() / 2.0;
  

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


