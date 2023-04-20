#ifndef UTILITY_H_
#define UTILITY_H_

#include <iostream>
#include <algorithm>
#include <cmath>
// #include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2/utils.h>
#include <tf2/transform_datatypes.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <wombat_utility/wombat_types.hpp>
#include <wombat_utility/wombatPolygon.hpp>

#include <nav2_costmap_2d/footprint.hpp>


namespace wombat{




class Utility{
public:

// static inline std

static inline nav_msgs::msg::Path checkPath(const nav_msgs::msg::Path& path)
{
  nav_msgs::msg::Path ret = path;
  auto frame_id = path.header.frame_id;
  auto stamp = path.header.stamp;

  //check if updateing frameid and stamp is needed
  if(path.poses.empty())
    return ret;
  
  //only check last pose
  if(!path.poses.back().header.frame_id.empty())
    return ret;

  for(auto& e : ret.poses)
  {
    e.header.frame_id = frame_id;
    e.header.stamp = stamp;
  }
  return ret;
}


static inline geometry_msgs::msg::TwistStamped getEmptyTwist(const rclcpp::Time& stamp, const std::string& frame_id)
{
  geometry_msgs::msg::TwistStamped ret;
  ret.header.stamp = stamp;
  ret.header.frame_id = frame_id;

  ret.twist.linear.x  = 0.0;
  ret.twist.linear.y  = 0.0;
  ret.twist.linear.z  = 0.0;
  ret.twist.angular.x = 0.0;
  ret.twist.angular.y = 0.0;
  ret.twist.angular.z = 0.0;

  return ret;
}


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




static inline Eigen::Rotation2Dd angleBetween(const Eigen::Vector2d vec_a, const Eigen::Vector2d vec_b)
{
  double dot = vec_a.dot(vec_b);
  double det = vec_a.x() * vec_b.y() - vec_a.y() * vec_b.x();
  double angle = ::atan2(det, dot);
  
  return Eigen::Rotation2Dd(angle);
}



static inline double computePathLength(const nav_msgs::msg::Path& path)
{
  double dist = 0.0;
  for(unsigned int i = 1; i < path.poses.size(); i++)
  {
    dist += (Convert::toVector2d(path.poses[i].pose.position) - Convert::toVector2d(path.poses[i-1].pose.position)).norm();
  }
  return dist;
}


static inline bool isEqual(const double lhs, const double rhs, const double epsilon = 0.00001)
{
  return std::abs(lhs - rhs) < epsilon;
}

static inline bool isEqual(const Pose2& lhs, const Pose2& rhs, const double epsilon = 0.00001)
{
  return isEqual(lhs.position.x(), rhs.position.x(), epsilon) && isEqual(lhs.position.y(), rhs.position.y(), epsilon);
}



static inline Polygon2d createFootprint(const std::string& footprint)
{
  Polygon2d polygon;
  std::vector<geometry_msgs::msg::Point> ros_points;
  auto ret = nav2_costmap_2d::makeFootprintFromString(footprint, ros_points);
  if(!ret)
  {
    RCLCPP_ERROR(rclcpp::get_logger("wombat_utility"), "Failed to parse footprint string");
    return polygon;
  }

  // std::cout << "points: " << std::endl;
  // for(auto& e : ros_points)
  // {
  //   std::cout << "p: " << e.x << ", " << e.y << std::endl;
  // }
  polygon.fromRosPoints(ros_points);

  return polygon;
}


static inline Polygon2d circleToPolygon(const double radius, const unsigned int num_points)
{
  Polygon2d polygon;
  for(unsigned int i = 0; i < num_points; i++)
  {
    double angle = 2.0 * M_PI * i / num_points;
    polygon.push_back( Point2(radius * std::cos(angle), radius * std::sin(angle)) );
  }
  return polygon;
}


};











} //namespace wombat

#endif  //UTILITY_H_