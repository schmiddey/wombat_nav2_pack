#ifndef WOMBATLOCALPATH2_H_
#define WOMBATLOCALPATH2_H_

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


#include "wombat_utility/utility.hpp"
#include "wombat_utility/wombatPath2.hpp"

namespace wombat{


class LocalPath2{
public:
  explicit LocalPath2(const Path2& global_path, const double target_dist, const double max_dist) : 
    _path(global_path),
    _target_dist(target_dist),
    _max_dist(max_dist)
  {  
    _it_begin  = _path.poses().begin();
    _it_target = _path.poses().begin();
    _it_end    = _path.poses().begin();
  }

  ~LocalPath2() = default;

  
  /**
   * @brief update local path 
   * 
   */
  void update(const Pose2& robot_pose)
  {
    this->determineLocalPath(robot_pose);
  }


  double localPathLength_from_begin() const
  {
    return _path.computePathLength(_it_begin, _it_end);
  }
  double localPathLength_from_target() const
  {
    return _path.computePathLength(_it_target, _it_end);
  }

  const Path2& path() const
  { 
    return _path;
  }

  Path2& path()
  { 
    return _path;
  }

  Path2 extractLocalPath() const
  {
    return _path.subset(_it_begin, _it_end);
  }

  const Pose2& getnearestPose() const
  {
    return *_it_begin;
  }

  const Pose2& getLocalTarget() const 
  {
    return *_it_target;
  }

  const Pose2& getFinalTarget() const
  {
    return _path.poses().back();
  }

  //iterators
  const std::vector<Pose2>::const_iterator iteratorLocalBegin() const
  {
    return _it_begin;
  }

  const std::vector<Pose2>::const_iterator iteratorLocalTarget() const
  {
    return _it_target;
  }

  const std::vector<Pose2>::const_iterator iteratorLocalEnd() const
  {
    return _it_end;
  }


  bool empty() const
  {
    return _path.empty();
  }


private:
  void determineLocalPath(const Pose2& robot_pose)
  {
    _last_robot_pose = robot_pose;

    //find first element further than target_dist (starting at the begin of the path)
    auto sq_target_dist = _target_dist * _target_dist;
    _it_target = std::find_if(_it_begin, _path.poses().end(),
                              [&](const Pose2& plan_pose) -> bool{
                                return (robot_pose.position - plan_pose.position).squaredNorm() >= sq_target_dist;
                              });

    if(_it_target == _path.poses().end() && !_path.empty())
    {
      std::cout << "---- reached end (target)" << std::endl;
      --_it_target;
    }


    //todo prove if element dist is much greater than target dist, then interpolate (when path has not many elements)

    //find nearest element to robot pos
    //iterate over all elements from begin until transformation_begin (element at target range) and find element with min dist
    double min_dist = sq_target_dist * 4;
    auto it_min = _it_begin;
    for(auto it = _it_begin; it != (_it_target+1); ++it) 
    {
      auto sq_dist = (robot_pose.position - it->position).squaredNorm();
      if(sq_dist < min_dist)
      {
        it_min = it;
        min_dist = sq_dist;
      }
    }
    _it_begin = it_min;



    //find first element further than max_dist //todo maybe prove if is is costmap rect
    auto sq_max_dist = _max_dist * _max_dist;
    _it_end = std::find_if(_it_begin, _path.poses().end(),
                           [&](const Pose2& plan_pose) -> bool{
                             return (robot_pose.position - plan_pose.position).squaredNorm() >= sq_max_dist;
                           });

    if(_it_end == _path.poses().end() && !_path.empty())
    {
      std::cout << "+++++++++ reached end (end)" << std::endl;
      --_it_end;
    }

  //debug
  auto dist_to_target = (robot_pose.position - _it_target->position).norm();

    for(unsigned int i = 0; i < _path.size(); i++)
    {
      std::cout << i << ": " << _path.poses()[i] << std::endl;
    }

    std::cout << "size _path: " << _path.size() << std::endl;
    std::cout << "_it_begin: " << std::distance(_path.poses().begin(), _it_begin) << std::endl;
    std::cout << "_it_target: " << std::distance(_path.poses().begin(), _it_target) << std::endl;
    std::cout << "_it_end: " << std::distance(_path.poses().begin(), _it_end) << std::endl;
    std::cout << "dist_to_target: " << dist_to_target << std::endl;
  }

private:
  Path2 _path;

  Pose2 _last_robot_pose; //todo check if needed
  
  double _target_dist = 0.2;
  double _max_dist    = 10.0;

  

  std::vector<wombat::Pose2>::iterator _it_begin ;   //< iterator of path element nearest to robot (always towards target)
  std::vector<wombat::Pose2>::iterator _it_target;   //< iterator of path element at defined target range, first element past target range
  std::vector<wombat::Pose2>::iterator _it_end   ;   //< iterator of path element at the end of the local path, fist element past max range

};




} //namespace wombat

#endif  //WOMBATLOCALPATH2_H_