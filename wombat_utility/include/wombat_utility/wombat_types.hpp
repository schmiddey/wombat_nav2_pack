#ifndef WOMBAT_TYPES_H_
#define WOMBAT_TYPES_H_

#include <iostream>
#include <algorithm>
#include <ostream>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav2_costmap_2d/costmap_2d.hpp>
#include <tf2/utils.h>
#include <tf2/transform_datatypes.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace wombat
{

using Point2 = Eigen::Vector2d;


//forwards
class Pixel2;


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

static inline Point2 fromRosPoint(const geometry_msgs::msg::Point& point)
{
  return Point2(point.x, point.y);
}

static inline geometry_msgs::msg::Point toRosPoint(const Point2& vec)
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

// static inline nav2_costmap_2d::MapLocation toMaplocation(const Pixel2 pixel)
// {
//   nav2_costmap_2d::MapLocation map_location;
//   map_location.x = pixel.x();
//   map_location.y = pixel.y();
//   return map_location;
// }

// static inline Pixel2 toPixel2(const nav2_costmap_2d::MapLocation map_location)
// {
//   return Pixel2(map_location.x, map_location.y);


};

// types
struct Pose2{
  Pose2() = default;
  Pose2(const Pose2&) = default;
  Pose2(Pose2&&) = default;
  Pose2& operator=(const Pose2&) = default;
  Pose2& operator=(Pose2&&) = default;
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




class Pixel2{
public:
  using value_type = int;

  Pixel2() = default;
  Pixel2(const value_type x, const value_type y):
    _x(x),
    _y(y)
  { }
  Pixel2(const nav2_costmap_2d::MapLocation& map_loc):
    _x(static_cast<value_type>(map_loc.x)),
    _y(static_cast<value_type>(map_loc.y))
  { } 
  ~Pixel2() = default;

  //copy and move constructors
  Pixel2(const Pixel2& other) = default;
  Pixel2(Pixel2&& other) = default;
  Pixel2& operator=(const Pixel2& rhs) = default;
  Pixel2& operator=(Pixel2&& rhs) = default;



  value_type x() const {return _x;}
  value_type y() const {return _y;}
  
  value_type& x() {return _x;}
  value_type& y() {return _y;}

  Pixel2 operator+(const Pixel2& rhs)
  {
    return Pixel2(_x + rhs.x(), _y + rhs.y());
  }

  Pixel2 operator-(const Pixel2& rhs)
  {
    return Pixel2(_x - rhs.x(), _y - rhs.y());
  }

  Pixel2 operator*(const double& rhs)
  {
    return Pixel2(std::round(_x * rhs), std::round(_y * rhs));
  }
  
  Pixel2& operator+=(const Pixel2& rhs)
  {
    _x += rhs.x();
    _y += rhs.y();
    return *this;
  }

  Pixel2& operator-=(const Pixel2& rhs)
  {
    _x -= rhs.x();
    _y -= rhs.y();
    return *this;
  }

  Pixel2& operator*=(const double& rhs)
  {
    _x = std::round(_x * rhs);
    _y = std::round(_x * rhs);
    return *this;
  }


  bool operator==(const Pixel2& rhs) const
  {
    return _x == rhs.x() && _y == rhs.y();
  }

  bool operator!=(const Pixel2& rhs) const
  {
    return !(*this == rhs);
  }

  nav2_costmap_2d::MapLocation toMapLocation() const
  {
    nav2_costmap_2d::MapLocation map_loc;
    map_loc.x = _x;
    map_loc.y = _y;
    return map_loc;
  }


public: // static
  static inline std::vector<Pixel2> fromMapLocationList(const std::vector<nav2_costmap_2d::MapLocation>& ml_list)
  {
    std::vector<Pixel2> p_list;
    p_list.reserve(ml_list.size());
    for(const auto& ml : ml_list)
    {
      p_list.emplace_back(ml);
    }
    return p_list;
  }

  static inline std::vector<nav2_costmap_2d::MapLocation> toMapLocationList(const std::vector<Pixel2>& p_list)
  {
    std::vector<nav2_costmap_2d::MapLocation> ml_list;
    ml_list.reserve(p_list.size());
    for(const auto& p : p_list)
    {
      ml_list.emplace_back(p.toMapLocation());
    }
    return ml_list;
  }

private:
  value_type _x;
  value_type _y;
};

template<class P>
class Rect2{
public:
  Rect2() = default;
  Rect2(const P& p, typename P::value_type width, typename P::value_type height):
    _p(p),
    _width(width),
    _height(height)
  { }



  P p() const {return _p;}
  typename P::value_type width() const {return _width;}
  typename P::value_type height() const {return _height;}

private:
  P _p;
  typename P::value_type _width;
  typename P::value_type _height;
};





} // namespace wombat

//overload output operator
std::ostream& operator<<(std::ostream& os, const wombat::Pose2& pose)
{
  os << "(" << pose.position.x() << ", " << pose.position.y() << ")";
  return os;
}

std::ostream& operator<<(std::ostream& os, const wombat::Pixel2& pixel)
{
  os << "(" << pixel.x() << ", " << pixel.y() << ")";
  return os;
}

#endif  //WOMBAT_TYPES_H_