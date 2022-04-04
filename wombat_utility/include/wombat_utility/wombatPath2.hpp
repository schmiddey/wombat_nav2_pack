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
    ros_path.poses.reserve(_poses.size());

    for(const auto& pose : _poses)
    {
      geometry_msgs::msg::PoseStamped ros_pose;
      ros_pose.header = _header;
      ros_pose.pose = pose.toRosPose();
      ros_path.poses.push_back(ros_pose);
    }

    assert(ros_path.poses.size() == _poses.size());

    return ros_path;
  }

  void fromRosPath(const nav_msgs::msg::Path& ros_path)
  {
    _header = ros_path.header;
    _poses.reserve(ros_path.poses.size());

    for(auto& e : ros_path.poses)
    {
      _poses.push_back(Pose2(e.pose));
    }

    assert(ros_path.poses.size() == _poses.size());
  }

  /**
   * @brief computes path length beween given iterators
   * 
   * @return double 
   */
  double computePathLength(const std::size_t idx_begin = 0, const std::size_t idx_end = 0) const
  {
    //bounds are checked in this function
    return this->computePathLength(_poses.begin() + idx_begin, _poses.begin() + idx_end);
  }
  
  double computePathLength(const std::vector<Pose2>::const_iterator it_begin, const std::vector<Pose2>::const_iterator it_end) const
  {
    double dist = 0.0;
    auto tmp_it_end = it_end + 1; //+1 because we want to include the last pose
    //check bounds
    if(tmp_it_end > _poses.end())
    {
      tmp_it_end = _poses.end();
    }
    if(it_begin >= tmp_it_end)
    {
      return dist;  //return zero dist
    }

    for(auto it = it_begin + 1; it != tmp_it_end; ++it)
    {
      dist += (it->position - (it-1)->position).norm();
    }
    return dist;
  }

  /**
   * @brief returns a path defined by given iterators
   * 
   */
  Path2 subset(const std::size_t idx_begin = 0, const std::size_t idx_end = 0) const
  {
    auto tmp_idx_begin = idx_begin;
    auto tmp_idx_end = idx_end;
    if(idx_end >= _poses.size() || idx_end == 0)
    {
      tmp_idx_end = _poses.size() - 1;
    }

    return this->subset(_poses.begin() + tmp_idx_begin, _poses.begin() + tmp_idx_end);
  }

  /**
   * @brief returns a path defined by given iterators
   * 
   */
  Path2 subset(const std::vector<Pose2>::const_iterator it_begin, const std::vector<Pose2>::const_iterator it_end) const
  {
    Path2 sub_path;
    sub_path._header = this->_header;

    auto tmp_it_end = it_end + 1; //+1 because we want to include the given element
    //check bounds
    if(tmp_it_end == _poses.end())
    {
      std::cout << "+++++++ qeual than end in subset" << std::endl;
    }

    if(tmp_it_end > _poses.end())
    {
      std::cout << "+++++++ greater than end in subset" << std::endl;
      tmp_it_end = _poses.end();
    }
    if(it_begin >= tmp_it_end)
    {
      return sub_path;  //return empty path
    }
    
    try
    {
      sub_path._poses = decltype(_poses)(it_begin, tmp_it_end);
    }
    catch(const std::exception& e)
    {
      // std::cerr << e.what() << '\n';
      sub_path.poses().clear(); 
      return sub_path;  //return empty path
    }
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

  std::size_t size() const
  {
    return _poses.size();
  }

  bool empty() const
  {
    return _poses.empty();
  }

private:
  std_msgs::msg::Header _header;
  std::vector<Pose2> _poses;

};



} // namespace wombat

#endif  //WOMBATPATH2_H_


