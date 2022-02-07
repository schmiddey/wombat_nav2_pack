#ifndef MECANUM_CONTROLLER_H_
#define MECANUM_CONTROLLER_H_

#include <functional>
#include <cmath>
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
  
  virtual void initialize() override
  {
    //todo params

    if(_orientation_target == "by_next")
    {
      _fcn_computeAngularVel = std::bind(&MecanumController::computeAngularVel_by_next, this, std::placeholders::_1, std::placeholders::_2);
    }
    else if(_orientation_target == "by_path")
    {
      _fcn_computeAngularVel = std::bind(&MecanumController::computeAngularVel_by_path, this, std::placeholders::_1, std::placeholders::_2);
    }
    else
    {//fallback as by next
      //todo log error!
      RCLCPP_ERROR(_logger, "unknown mode for orientation target is given: %s ... will use by_next", "todo");
      _fcn_computeAngularVel = std::bind(&MecanumController::computeAngularVel_by_next, this, std::placeholders::_1, std::placeholders::_2);
    }
  }

  virtual geometry_msgs::msg::TwistStamped control(const geometry_msgs::msg::PoseStamped& robot_pose,
                                                   const nav_msgs::msg::Path& local_path,
                                                   const double end_approach_scale) override
  {
    Pose2 r_pose(robot_pose.pose);

    Pose2 t_pose(local_path.poses.front().pose);

    auto t_vec = t_pose.position - r_pose.position;
    
    auto vel_lin = t_vec.normalized(); //todo set max vel to 1.0
    vel_lin *= end_approach_scale; //todo may use an scale fcn

    //roate vel_lin by robot orientaion
    Eigen::Rotation2Dd neg_r_rot(r_pose.rotation.smallestAngle() * -1);
    vel_lin = neg_r_rot * vel_lin;
    
    double vel_ang = 0.0;

    if(_fcn_computeAngularVel)
    {
      vel_ang = _fcn_computeAngularVel(r_pose, t_pose).smallestAngle();
    }
    else
    {
      RCLCPP_ERROR(_logger, "function object for computing angluar vel is not valid... vel_ang is set to 0.0");
    }

    //apply wait_for_rot
    vel_lin *= this->waitForRotationTransfere(vel_ang, _wait_for_rotation);
    

    geometry_msgs::msg::TwistStamped cmd;
    cmd.header.frame_id = "todo";
    cmd.header.stamp    = robot_pose.header.stamp; //todo

    cmd.twist.linear.x = vel_lin.x();
    cmd.twist.linear.y = vel_lin.y();
    cmd.twist.angular.z = vel_ang;
    return cmd;
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
  double _wait_for_rotation = 1.0;  ///< value 0.0..1.0  -> 0: finishes rotation bevor moving linear, 1: moves linear undependend from rotation status
  std::string _orientation_target = "by_next";  ///< target of orientation while moving: by orientation defined in path -> "by_path", toward next path element -> "by_next"
};

} //namespace wombat

#endif  //MECANUM_CONTROLLER_H_