#ifndef UTILITY_H_
#define UTILITY_H_

#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2/utils.h"
#include "Eigen/Dense"
#include "Eigen/Geometry"

namespace wombat{




class Utility{
public:

// static void constrain(/*todo*/)
// {
//   //todo
// }

// static double rescale(double val_in, double in_min, double in_max, double out_min, double out_max)
// {
//   //todo
//   return 0.0;
// }


static double computeSquareDistance2D(const geometry_msgs::msg::PoseStamped& pose_a,
                                      const geometry_msgs::msg::PoseStamped& pose_b)
{
  double x_diff = pose_a.pose.position.x - pose_b.pose.position.x;
  double y_diff = pose_a.pose.position.y - pose_b.pose.position.y;

  return x_diff * x_diff + y_diff * y_diff;
}

// static geometry_msgs::msg::PoseStamped createPoseStamped(const std::string& frame, 
//                                                          builtin_interfaces::msg::Time& stamp,
//                                                          )


//conversions
//Eigen
static Eigen::Vector2d toVector2d(const geometry_msgs::msg::Point& point)
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

static Eigen::Rotation2Dd angleBetween(const Eigen::Vector2d vec_a, const Eigen::Vector2d vec_b)
{
  double dot = vec_a.dot(vec_b);
  double det = vec_a.x() * vec_b.y() - vec_a.y() * vec_b.x();
  double angle = ::atan2(det, dot);
  
  return Eigen::Rotation2Dd(angle);
}

static Eigen::Rotation2Dd toRotation2d(const geometry_msgs::msg::Quaternion& quat)
{
  tf2::Quaternion tf_quat;
  tf2::fromMsg(quat, tf_quat);
  tf2::Matrix3x3 m(tf_quat);

  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  // Eigen::Rotation2Dd rot(Utility::normalizeAngle(yaw));
  return Eigen::Rotation2Dd(yaw);
}

static double computePathLength(const nav_msgs::msg::Path& path)
{
  double dist = 0.0;
  for(unsigned int i = 1; i < path.poses.size(); i++)
  {
    dist += (Utility::toVector2d(path.poses[i].pose.position) - Utility::toVector2d(path.poses[i-1].pose.position)).norm();
  }
  return dist;
}


};




//types
struct Pose2{
  Pose2() = default;
  Pose2(const Eigen::Vector2d& pos, const Eigen::Vector2d& ori_vec):
    position(pos),
    orientation_vec(ori_vec)
  { }
  explicit Pose2(const geometry_msgs::msg::Pose& ros_pose) : 
    position(Utility::toVector2d(ros_pose.position)),
    rotation(Utility::toRotation2d(ros_pose.orientation)),
    orientation_vec(rotation * Eigen::Vector2d(1,0))
  { }
  ~Pose2() = default;
  
  Eigen::Vector2d    position;
  Eigen::Rotation2Dd rotation;
  Eigen::Vector2d    orientation_vec;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


using Point2 = Eigen::Vector2d;

} //namespace wombat

#endif  //UTILITY_H_