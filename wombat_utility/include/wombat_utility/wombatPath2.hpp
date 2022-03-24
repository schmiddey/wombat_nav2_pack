#ifndef WOMBATPATH2_H_
#define WOMBATPATH2_H_

#include <iostream>
#include <algorithm>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2/utils.h"
#include "tf2/transform_datatypes.h"
#include "Eigen/Dense"
#include "Eigen/Geometry"

#include "wombat_utility/wombat_types.hpp"


namespace wombat
{

class Path2{
public:
  Path2() = default;
  explicit Path2(const nav_msgs::msg::Path& ros_path)
  {
    this->fromRosPath(ros_path);
  }

  Path2(const std_msgs::msg::Header& header, const std::vector<Pose2>& poses) :
    _header(header),
    _poses(poses)
  { }

  Path2(const Path2& rhs) = default;
  Path2(Path2&& rhs) = default;
  Path2& operator=(const Path2& rhs) = default;
  Path2& operator=(Path2&& rhs) = default;



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
  double computePathLength(const std::size_t idx_begin = 0) const
  {
    double dist = 0.0;
    for(unsigned int i = idx_begin + 1; i < _poses.size(); i++)
    {
      dist += (_poses[i].position - _poses[i-1].position).norm();
    }
    return dist;
  }

  /**
   * @brief returns a path defined by given iterators
   * 
   */
  Path2 subset(const std::size_t idx_begin = 0, const std::size_t idx_end = 0)
  {
    auto tmp_idx_begin = idx_begin;
    auto tmp_idx_end = idx_end;
    if(idx_end >= _poses.size() || idx_end == 0)
    {
      tmp_idx_end = _poses.size() - 1;
    }

    //todo prove bounds
    if(idx_begin >= idx_end)
    {
      tmp_idx_begin = 0;
    }

    Path2 sub_path;
    sub_path._header = this->_header;
    sub_path._poses = decltype(_poses)(_poses.begin() + tmp_idx_begin, _poses.begin() + tmp_idx_end);

    return sub_path;
  }

  const std::vector<Pose2>& poses() const
  {
    return _poses;
  }

  std::vector<Pose2>& poses()
  {
    return _poses;
  }

  const std_msgs::msg::Header& header() const
  {
    return _header;
  }

  std_msgs::msg::Header& header() 
  {
    return _header;
  }

private:
  std_msgs::msg::Header _header;
  std::vector<Pose2> _poses;

};



} // namespace wombat

#endif  //WOMBATPATH2_H_