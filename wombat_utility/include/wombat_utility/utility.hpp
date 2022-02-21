#ifndef UTILITY_H_
#define UTILITY_H_

#include <iostream>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2/utils.h"
#include "Eigen/Dense"
#include "Eigen/Geometry"

namespace wombat{




class Utility{
public:

/**
 * @brief ala arduino
 * 
 * @param val 
 * @param min_val 
 * @param max_val 
 * @return double 
 */
static inline double constrain(const double val, const double min_val, const double max_val)
{
  return std::min(max_val, std::max(min_val, val));
}

/**
 * @brief ala arduino map
 * 
 * @param val_in 
 * @param in_min 
 * @param in_max 
 * @param out_min 
 * @param out_max 
 * @return double 
 */
static inline double rescale(double val_in, double in_min, double in_max, double out_min, double out_max)
{
  return (val_in - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


static inline double computeSquareDistance2D(const geometry_msgs::msg::Pose& pose_a,
                                             const geometry_msgs::msg::Pose& pose_b)
{
  double x_diff = pose_a.position.x - pose_b.position.x;
  double y_diff = pose_a.position.y - pose_b.position.y;

  return x_diff * x_diff + y_diff * y_diff;
}

// static geometry_msgs::msg::PoseStamped createPoseStamped(const std::string& frame, 
//                                                          builtin_interfaces::msg::Time& stamp,
//                                                          )


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

static inline Eigen::Rotation2Dd angleBetween(const Eigen::Vector2d vec_a, const Eigen::Vector2d vec_b)
{
  double dot = vec_a.dot(vec_b);
  double det = vec_a.x() * vec_b.y() - vec_a.y() * vec_b.x();
  double angle = ::atan2(det, dot);
  
  return Eigen::Rotation2Dd(angle);
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

static inline double computePathLength(const nav_msgs::msg::Path& path)
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

  geometry_msgs::msg::Pose toRosPose() const
  {
    geometry_msgs::msg::Pose ros_pose;
    tf2::Quaternion tf_quat;
    tf_quat.setRPY(0,0,this->rotation.smallestAngle()); //TODO check
    ros_pose.orientation = tf2::toMsg(tf_quat);
    ros_pose.position = Utility::toRosPoint(this->position);
    return ros_pose;
  }
  
  Eigen::Vector2d    position;
  Eigen::Rotation2Dd rotation;
  Eigen::Vector2d    orientation_vec;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


using Point2 = Eigen::Vector2d;


class Path2{
public:
  Path2() = default;
  explicit Path2(const nav_msgs::msg::Path& ros_path)
  {
    this->fromRosPath(ros_path);
  }
  ~Path2() = default;
  
  nav_msgs::msg::Path toRosPath() const
  {
    nav_msgs::msg::Path ros_path;
    ros_path.header = _header;
    std::transform(_poses.begin(), _poses.end(), ros_path.poses.begin(),
                   [&](const auto& pose) -> geometry_msgs::msg::PoseStamped {
                        geometry_msgs::msg::PoseStamped ret;
                        ret.header = _header;
                        ret.pose = pose.toRosPose();
                        return ret; 
                      });   

    return ros_path;
  }

  void fromRosPath(const nav_msgs::msg::Path& ros_path)
  {
    _header = ros_path.header;
    // _poses.reserve(ros_path.poses.size());
    // for(auto& e : ros_path.poses)
    // {
    //   _poses.push_back(Pose2(e.pose));
    // }
    std::transform(ros_path.poses.begin(), ros_path.poses.end(), _poses.begin(),
                   [](auto& ros_pose) -> Pose2 { return Pose2(ros_pose.pose); });

    assert(ros_path.poses.size() == _poses.size());

  }

  /**
   * @brief computes path length beween fiven iterators
   * 
   * @return double 
   */
  double computePathLength() const
  {
    return 0.0;
  }

  /**
   * @brief returns a path defined by given iterators
   * 
   */
  void subset()
  {

  }

private:
  std_msgs::msg::Header _header;
  std::vector<Pose2> _poses;

  int _it_near_pose;
  int _it_far_pose;

};


} //namespace wombat

#endif  //UTILITY_H_