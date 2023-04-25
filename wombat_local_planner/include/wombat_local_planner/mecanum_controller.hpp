#ifndef MECANUM_CONTROLLER_H_
#define MECANUM_CONTROLLER_H_

#include <functional>
#include <cmath>
#include <nav2_util/node_utils.hpp>

#include "base_controller.hpp"
#include "wombat_utility/utility.hpp"
#include "Eigen/Dense"
#include "Eigen/Geometry"


namespace wombat{


class MecanumController : public BaseController{
public:
  MecanumController() = default;
  virtual ~MecanumController() = default;

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
    nav2_util::declare_parameter_if_not_declared(node, plugin_name + ".orientation_offset", rclcpp::ParameterValue(15));
    nav2_util::declare_parameter_if_not_declared(node, plugin_name + ".orientation_target", rclcpp::ParameterValue(std::string("by_next")));
    
    // 
    node->get_parameter(plugin_name + ".angular_boost",        _angular_boost);
    node->get_parameter(plugin_name + ".linear_boost",         _linear_boost);
    node->get_parameter(plugin_name + ".wait_for_rotation",    _wait_for_rotation);
    node->get_parameter(plugin_name + ".max_vel_lin",          _max_vel_lin);
    node->get_parameter(plugin_name + ".max_vel_ang",          _max_vel_ang);
    node->get_parameter(plugin_name + ".orientation_offset",   _orientation_offset);
    node->get_parameter(plugin_name + ".orientation_target",   _orientation_target);

    RCLCPP_INFO(_logger, "angular_boost: %f", _angular_boost);
    RCLCPP_INFO(_logger, "linear_boost: %f", _linear_boost);
    RCLCPP_INFO(_logger, "wait_for_rotation: %f", _wait_for_rotation);
    RCLCPP_INFO(_logger, "max_vel_lin: %f m",       _max_vel_lin);
    RCLCPP_INFO(_logger, "max_vel_ang: %f m",       _max_vel_ang);
    RCLCPP_INFO(_logger, "orientation_offset: %d ", (int)_orientation_offset);
    RCLCPP_INFO(_logger, "orientation_target: %s", _orientation_target.c_str());
    
    
    // 
    RCLCPP_INFO(_logger, "--------------------------------------");
 

    this->setOrientationMode(_orientation_target);
  }

  virtual geometry_msgs::msg::Twist control(const Pose2& robot_pose,
                                            const std::shared_ptr<LocalPath2>& local_path,
                                            const double end_approach_scale,
                                            const Pose2& tolerance = Pose2()) override
  {
    // if(local_path.poses.empty())
      // RCLCPP_INFO(_logger, "local path size: %d", (int)local_path.poses.size());
    assert(!local_path->path().poses().empty());

    (void)tolerance;
    
    if(local_path->path().size() == 1)
    {
      this->setOrientationMode("by_path");
    }
    else
    {
      this->setOrientationMode(_orientation_target);
    }

    // Pose2 r_pose(robot_pose.pose);

    // Pose2 t_pose(local_path.poses.front().pose);
    auto t_pose = local_path->getLocalTarget();

    Eigen::Vector2d t_vec = t_pose.position - robot_pose.position;


    //todo for now no offset available
    //aply orientation offset (sets orientation from a future pose in path)
    // auto tmp_idx = std::min(_orientation_offset, (int)(local_path->path().size() - 1));
    auto it_offset_offset = local_path->iteratorLocalTarget() + _orientation_offset;
    if(it_offset_offset > local_path->iteratorLocalEnd())
    {
      it_offset_offset = local_path->iteratorLocalEnd();
    }

    t_pose.orientation_vec = it_offset_offset->orientation_vec;
    t_pose.rotation = it_offset_offset->rotation;


    // RCLCPP_INFO(_logger, "len t_vec: %f", t_vec.norm());
    // RCLCPP_INFO(_logger, "end_appr_scale: %f", end_approach_scale);
    
    auto vel_lin = t_vec.normalized(); //todo set max vel to 1.0
    vel_lin *= _linear_boost;
    if(vel_lin.norm() > 1.0)
    {
      vel_lin.normalize();
    }

    vel_lin *= end_approach_scale; //todo may use an scale fcn

    // vel_lin *= 0.2;
    //roate vel_lin by robot orientaion
    Eigen::Rotation2Dd neg_r_rot(robot_pose.rotation.smallestAngle() * -1);
    vel_lin = neg_r_rot * vel_lin;
    
    double vel_ang = 0.0;

    if(_fcn_computeAngularVel)
    {
      vel_ang = _fcn_computeAngularVel(robot_pose, t_pose).smallestAngle();
    }
    else
    {
      RCLCPP_ERROR(_logger, "function object for computing angluar vel is not valid... vel_ang is set to 0.0");
    } 
    
    //normalize angvel to -1.0 .. 1.0
    vel_ang = Utility::rescale(vel_ang, -M_PI, M_PI, -1.0, 1.0);

    //apply wait_for_rot
    vel_lin *= this->waitForRotationTransfere(vel_ang, _wait_for_rotation);
    
    //boost ang_vel
    vel_ang *= _angular_boost;
    vel_ang = Utility::constrain(vel_ang, -1.0, 1.0);

    geometry_msgs::msg::Twist cmd;

    //aply max vel
    vel_lin *= _max_vel_lin;
    vel_ang *= _max_vel_ang;

    cmd.linear.x = vel_lin.x();
    cmd.linear.y = vel_lin.y();
    cmd.angular.z = vel_ang;
    return cmd;
  }

virtual void setReverseMode(const bool reverse_mode) override
{
  (void)reverse_mode;
}

virtual bool getReverseMode()  const override
{
  return false;
}
 
protected: //functions
  Eigen::Rotation2Dd computeAngularVel_by_next(const Pose2& robot_pose, const Pose2& target_pose) const
  {
    auto t_vec = target_pose.position - robot_pose.position;

    return Utility::angleBetween(robot_pose.orientation_vec, t_vec);
  }
  
  Eigen::Rotation2Dd computeAngularVel_by_path(const Pose2& robot_pose, const Pose2& target_pose) const
  {
    return Utility::angleBetween(robot_pose.orientation_vec, target_pose.orientation_vec);
  }

  void setOrientationMode(const std::string& mode)
  {
    if(mode == "by_next")
    {
      _fcn_computeAngularVel = std::bind(&MecanumController::computeAngularVel_by_next, this, std::placeholders::_1, std::placeholders::_2);
    }
    else if(mode == "by_path")
    {
      _fcn_computeAngularVel = std::bind(&MecanumController::computeAngularVel_by_path, this, std::placeholders::_1, std::placeholders::_2);
    }
    else
    {//fallback as by next
      RCLCPP_ERROR(_logger, "unknown mode for orientation target is given: %s ... will use by_next", "todo");
      _fcn_computeAngularVel = std::bind(&MecanumController::computeAngularVel_by_next, this, std::placeholders::_1, std::placeholders::_2);
    }
  }


  /**
   * @brief transfere function for linear speed depending on angular speed
   * 
   * @todo directly prove vel_ang if expensive calc is needed 
   * 
   * @param vel_ang 
   * @param scale 
   * @return double 
   */
  double waitForRotationTransfere(const double vel_ang, const double scale) const
  {
    double arg_cos = 100 * (1 - std::pow(scale, 0.1)) * vel_ang;
    if(arg_cos > M_PI)
    {
      return 0;
    }
    return 0.5 + 0.5 * std::cos(arg_cos);
  }

protected: 
  

  std::function<Eigen::Rotation2Dd(const Pose2&, const Pose2&)> _fcn_computeAngularVel;

  // params
  double      _angular_boost     = 1.0;
  double      _linear_boost      = 1.0;
  double      _wait_for_rotation = 1.0; ///< value 0.0..1.0  -> 0: finishes rotation bevor moving linear, 1: moves linear undependend from rotation status
  double      _max_vel_lin       = 1.0;
  double      _max_vel_ang       = 1.0; 
  int         _orientation_offset = 15;
  std::string _orientation_target = "by_next";  ///< target of orientation while moving: by orientation defined in path -> "by_path", toward next path element -> "by_next"
};

} //namespace wombat

#endif  //MECANUM_CONTROLLER_H_