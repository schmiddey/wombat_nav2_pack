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

  Path2 getLocalPath() const
  {
    std::vector<Pose2> local_path(_it_begin, _it_end);

    return Path2(_path.header(), local_path);
  }

  const Pose2& getTarget() const 
  {
    return *_it_target;
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

    //todo prove if element dist is much greater than target dist, then interpolate (when path has not many elements)

    //find nearest element to robot pos
    //iterate over all elements from begin until transformation_begin (element at target range) and find element with min dist
    double min_dist = sq_target_dist * 4;
    auto it_min = _it_begin;
    for(auto it = _it_begin; it != _it_target; ++it)
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
    
  }

private:
  Path2 _path;

  Pose2 _last_robot_pose; //todo check if needed
  
  double _target_dist = 0.2;
  double _max_dist    = 10.0;

  

  decltype(_path.poses().begin()) _it_begin ;   //< iterator of path element nearest to robot (always towards target)
  decltype(_path.poses().begin()) _it_target;   //< iterator of path element at defined target range, first element past target range
  decltype(_path.poses().begin()) _it_end   ;   //< iterator of path element at the end of the local path, fist element past max range

};




} //namespace wombat

#endif  //WOMBATLOCALPATH2_H_