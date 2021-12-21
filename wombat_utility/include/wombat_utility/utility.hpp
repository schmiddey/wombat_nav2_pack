#ifndef UTILITY_H_
#define UTILITY_H_

#include<iostream>

#include"rclcpp/rclcpp.hpp"
#include"geometry_msgs/msg/pose_stamped.hpp"


namespace wombat{

class Utility{
public:

static void constrain(/*todo*/)
{
  //todo
}


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

};


} //namespace wombat

#endif  //UTILITY_H_