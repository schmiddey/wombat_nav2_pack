#ifndef WOMBAT_TYPES_H_
#define WOMBAT_TYPES_H_

#include <iostream>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2/utils.h"
#include "tf2/transform_datatypes.h"
#include "Eigen/Dense"
#include "Eigen/Geometry"

namespace wombat
{
class Convert{
public:
  
//conversions
//Eigen
static inline Eigen::Vector2d toVector2d(const geometry_msgs::msg::Point& point)
{
  return Eigen::Vector2d(point.x, point.y);
}

/**
 * @brief normalizes angle to -PI .. PI
 * 
 * @todo check use eigen::rotation2 -> smalestAngle ???
 * 
 * @param angle 
 * @return double 
 */
// static double normalizeAngle(const double angle)
// {
//   double ang = angle;

//   while(ang > M_PI) ang -= 2.0 * M_PI;
//   while(ang <= -M_PI) ang += 2.0 * M_PI;

//   return ang;
// }

static inline geometry_msgs::msg::Point toRosPoint(const Eigen::Vector2d& vec)
{
  geometry_msgs::msg::Point point;
  point.x = vec.x();
  point.y = vec.y();
  point.z = 0.0;
  return point;
}

static inline Eigen::Rotation2Dd toRotation2d(const geometry_msgs::msg::Quaternion& quat)
{
  tf2::Quaternion tf_quat;
  tf2::fromMsg(quat, tf_quat);
  tf2::Matrix3x3 m(tf_quat);

  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  // Eigen::Rotation2Dd rot(Utility::normalizeAngle(yaw));
  return Eigen::Rotation2Dd(yaw);
}

};


// types
struct Pose2{
  Pose2() = default;
  Pose2(const Eigen::Vector2d& pos, const Eigen::Vector2d& ori_vec):
    position(pos),
    orientation_vec(ori_vec)
  { }
  explicit Pose2(const geometry_msgs::msg::Pose& ros_pose) : 
    position(Convert::toVector2d(ros_pose.position)),
    rotation(Convert::toRotation2d(ros_pose.orientation)),
    orientation_vec(rotation * Eigen::Vector2d(1,0))
  { }
  ~Pose2() = default;

  geometry_msgs::msg::Pose toRosPose() const
  {
    geometry_msgs::msg::Pose ros_pose;
    tf2::Quaternion tf_quat;
    tf_quat.setRPY(0,0,this->rotation.smallestAngle()); //TODO check
    ros_pose.orientation = tf2::toMsg(tf_quat);
    ros_pose.position = Convert::toRosPoint(this->position);
    return ros_pose;
  }
  
  Eigen::Vector2d    position;
  Eigen::Rotation2Dd rotation;
  Eigen::Vector2d    orientation_vec;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


using Point2 = Eigen::Vector2d;





} // namespace wombat

//overload output operator
std::ostream& operator<<(std::ostream& os, const wombat::Pose2& pose)
{
  os << "(" << pose.position.x() << ", " << pose.position.y() << ")";
  return os;
}


#endif  //WOMBAT_TYPES_H_