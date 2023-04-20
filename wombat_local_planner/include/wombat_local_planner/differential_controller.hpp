#pragma once

#include <functional>
#include <cmath>
#include <nav2_util/node_utils.hpp>

#include "base_controller.hpp"
#include "wombat_utility/utility.hpp"
#include "Eigen/Dense"
#include "Eigen/Geometry"

// namespace ros2 = rclcpp;

namespace wombat{

class DifferentialController: public BaseController{
public:
  DifferentialController() = default;
  virtual ~DifferentialController() = default;

  //inherrit constructor
  using BaseController::BaseController;

  virtual void initialize(const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent,
                          const std::string& name) override
  {
    auto node = parent.lock();
    std::string plugin_name = name + ".controller";

    RCLCPP_INFO(_logger, "--------------------------------------");
    RCLCPP_INFO(_logger, "--    wombatController Params:      --");
    RCLCPP_INFO(_logger, "--------------------------------------");


    //params
    nav2_util::declare_parameter_if_not_declared(node, plugin_name + ".angular_boost",      rclcpp::ParameterValue(1.0));
    nav2_util::declare_parameter_if_not_declared(node, plugin_name + ".linear_boost",       rclcpp::ParameterValue(1.0));
    nav2_util::declare_parameter_if_not_declared(node, plugin_name + ".wait_for_rotation",  rclcpp::ParameterValue(1.0));
    nav2_util::declare_parameter_if_not_declared(node, plugin_name + ".max_vel_lin",        rclcpp::ParameterValue(1.0));
    nav2_util::declare_parameter_if_not_declared(node, plugin_name + ".max_vel_ang",        rclcpp::ParameterValue(1.0));
    
    // 
    node->get_parameter(plugin_name + ".angular_boost",        _angular_boost);
    node->get_parameter(plugin_name + ".linear_boost",         _linear_boost);
    node->get_parameter(plugin_name + ".wait_for_rotation",    _wait_for_rotation);
    node->get_parameter(plugin_name + ".max_vel_lin",          _max_vel_lin);
    node->get_parameter(plugin_name + ".max_vel_ang",          _max_vel_ang);

    //check _wait_for_rotation
    _wait_for_rotation = Utility::constrain(_wait_for_rotation, 0.01, 1.0);
    _angular_boost = Utility::constrain(_angular_boost, 0.0, 0.99);
    _linear_boost = Utility::constrain(_linear_boost, 0.0, 0.99);

    this->updateWaitForRotation(_wait_for_rotation);

    RCLCPP_INFO(_logger, "angular_boost: %f", _angular_boost);
    RCLCPP_INFO(_logger, "linear_boost: %f", _linear_boost);
    RCLCPP_INFO(_logger, "wait_for_rotation: %f", _wait_for_rotation);
    RCLCPP_INFO(_logger, "max_vel_lin: %f m",       _max_vel_lin);
    RCLCPP_INFO(_logger, "max_vel_ang: %f m",       _max_vel_ang);
    
    
    // 
    RCLCPP_INFO(_logger, "--------------------------------------");
 

  }
  
  virtual geometry_msgs::msg::Twist control(const Pose2& robot_pose,
                                            const std::shared_ptr<LocalPath2>& local_path,
                                            const double end_approach_scale,
                                            const Pose2& tolerance = Pose2()) override
  {
    assert(!local_path->path().poses().empty());
    
    // (void)tolerance;
    
    //print end_approach_scale
    RCLCPP_INFO(_logger, "end_approach_scale: %f", end_approach_scale);


    auto t_pose = local_path->getLocalTarget();
    Eigen::Vector2d t_vec = t_pose.position - robot_pose.position;

    auto g_pose = local_path->getFinalTarget();
    Eigen::Vector2d g_vec = g_pose.position - robot_pose.position;

    if(end_approach_scale == 1.0)
    {
      //log info
      RCLCPP_INFO(_logger, "reset end rotation");
      _do_endrotation = false;
    }
    else if(g_vec.norm() < tolerance.position.x())
    //implicite scale for tolerance this conroller makes tol 1.41 (sqrt(2)) time smaller
    //todo add a param for that and not use sqrt(2) fac ...
    {
      //log info
      RCLCPP_INFO(_logger, "end rotation");
      _do_endrotation = true;
    }

    if(_do_endrotation)
    {
      // t_vec = g_vec; //fucking wrong !!! must be orientation of goal!!!!
      t_vec = g_pose.orientation_vec;
    }

    //angle between pi and -pi
    auto angle = Utility::angleBetween(robot_pose.orientation_vec, t_vec).smallestAngle();

    //print angle
    RCLCPP_INFO(_logger, "angle: %f", angle);


    double vel_lin = 0.0;

    //aply transfere function to lin vel
    //linvel in diff controller is end_approach_scale (always 1.0, decreases near goal)
    if(!_do_endrotation)
    {
      vel_lin = this->angularTransfereFcn(angle, _ang_transfere_fac);
      vel_lin *= end_approach_scale;
    }

    //scale angular vel to -1.0 to 1.0
    auto vel_ang = Utility::rescale(angle, -M_PI, M_PI, -1.0, 1.0);

    //apply boost
    vel_ang = this->boostVel(vel_ang, _angular_boost);
    vel_lin = this->boostVel(vel_lin, _linear_boost);
    
    //limit vel
    vel_ang = Utility::constrain(vel_ang, -1.0, 1.0);
    vel_lin = Utility::constrain(vel_lin, 0.0, 1.0);
    
    //apply max_vel
    vel_ang *= _max_vel_ang;
    vel_lin *= _max_vel_lin;

    geometry_msgs::msg::Twist cmd;

    cmd.linear.x = vel_lin;
    cmd.angular.z = vel_ang;
    
    return cmd;
  }


private:

  double angularTransfereFcn(const double angle, const double fac)
  {
    // fac/(math.pow(ang, 2) + fac)
    return fac / (angle * angle + fac);
  }

  double boostVel(const double vel, const double fac)
  {
    return std::copysign(std::pow(std::abs(vel), 1-fac), vel);
  }

  void updateWaitForRotation(const double new_value)
  {
    _wait_for_rotation = new_value;
    _ang_transfere_fac = 1.0/ std::pow(2.0 * _wait_for_rotation, 10.0);
  }

private:

  double _angular_boost;
  double _linear_boost;
  double _wait_for_rotation;
  double _max_vel_lin;
  double _max_vel_ang;

  double _ang_transfere_fac = 0.0;

  bool _do_endrotation = false;
};



} //namespace wombat