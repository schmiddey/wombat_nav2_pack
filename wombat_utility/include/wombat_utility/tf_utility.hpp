#ifndef TF_UTILITY_H_
#define TF_UTILITY_H_

#include<optional>

#include"rclcpp/rclcpp.hpp"
#include"geometry_msgs/msg/pose_stamped.hpp"
#include"nav_msgs/msg/path.hpp"

#include"tf2_ros/buffer.h"

namespace wombat
{

class TfUtility{
public:

//todo do good transform by reference !!!
// static bool transformByRef(const std::shared_ptr<tf2_ros::Buffer> tf,
//                          geometry_msgs::msg::PoseStamped& pose, 
//                          const std::string& frame,
//                          const rclcpp::Duration& tf_tolerance)
// {
//   if(pose.header.frame_id == frame)
//   {
//     return true;
//   }

//   try
//   {
//     auto pose_transformed = tf->transform(pose, frame);
//     pose = pose_transformed;
//     return true;
//   }
//   catch(const tf2::ExtrapolationException& e)
//   {
//     auto transform_backup = tf->lookupTransform(frame, pose.header.frame_id, tf2::TimePointZero);
//     if(rclcpp::Time(pose.header.stamp) - rclcpp::Time(transform_backup.header.stamp) > tf_tolerance)
//     {
//       //error msg
//       RCLCPP_ERROR(rclcpp::get_logger("tf_help"), 
//                    "Transform data too old when converting from %s to %s",
//                    pose.header.frame_id.c_str(), 
//                    frame.c_str()
//                    );
//       RCLCPP_ERROR(rclcpp::get_logger("tf_help"),
//                    "Data time: %ds %uns, Transform time: %ds %uns",
//                    pose.header.stamp.sec,
//                    pose.header.stamp.nanosec,
//                    transform_backup.header.stamp.sec,
//                    transform_backup.header.stamp.nanosec
//                    );
//       return false;
//     }
//     else
//     {
//       geometry_msgs::msg::PoseStamped out_pose;
//       tf2::doTransform(pose, out_pose, transform_backup);
//       pose = out_pose;
//       return true;
//     }
//   }
//   catch(tf2::TransformException& e)
//   {
//     RCLCPP_ERROR(rclcpp::get_logger("tf_help"),
//                  "Exception in transformPose: %s",
//                  e.what()
//                  );
//     return false;
//   }
//   return false;
// }

/**
 * @brief taken from nav2::dwb_local_planner 
 * 
 * @param tf 
 * @param pose 
 * @param frame 
 * @param tf_tolerance 
 * @return std::optional<geometry_msgs::msg::PoseStamped> 
 */
static std::optional<geometry_msgs::msg::PoseStamped> transform(const std::shared_ptr<tf2_ros::Buffer> tf,
                                                                const geometry_msgs::msg::PoseStamped& pose, 
                                                                const std::string& frame,
                                                                const rclcpp::Duration& tf_tolerance)
{
  if(pose.header.frame_id == frame)
  {
    return pose;
  }

  try
  {
    auto pose_transformed = tf->transform(pose, frame);
    return pose_transformed;
  }
  catch(const tf2::ExtrapolationException& e)
  {
    auto transform_backup = tf->lookupTransform(frame, pose.header.frame_id, tf2::TimePointZero);
    if(rclcpp::Time(pose.header.stamp) - rclcpp::Time(transform_backup.header.stamp) > tf_tolerance)
    {
      //error msg
      RCLCPP_ERROR(rclcpp::get_logger("tf_help"), 
                   "Transform data too old when converting from %s to %s",
                   pose.header.frame_id.c_str(), 
                   frame.c_str()
                   );
      RCLCPP_ERROR(rclcpp::get_logger("tf_help"),
                   "Data time: %ds %uns, Transform time: %ds %uns",
                   pose.header.stamp.sec,
                   pose.header.stamp.nanosec,
                   transform_backup.header.stamp.sec,
                   transform_backup.header.stamp.nanosec
                   );
      return std::nullopt;
    }
    else
    {
      geometry_msgs::msg::PoseStamped out_pose;
      tf2::doTransform(pose, out_pose, transform_backup);
      return out_pose;
    }
  }
  catch(tf2::TransformException& e)
  {
    RCLCPP_ERROR(rclcpp::get_logger("tf_help"),
                 "Exception in transformPose: %s",
                 e.what()
                 );
    return std::nullopt;
  }
  return std::nullopt;
}

// static bool transformByRef(const std::shared_ptr<tf2_ros::Buffer> tf,
//                          nav_msgs::msg::Path& path, 
//                          const std::string& frame,
//                          const rclcpp::Duration& tf_tolerance)
// {
//   for(auto& e : path.poses)
//   {
//     auto t_pose = TfUtility::transform(tf, e, frame, tf_tolerance);
//     if(!t_pose)
//     {
//       //error
//       return false;
//     }
//     ret_path.poses.push_back(t_pose.value());
//   }
// }

static std::optional<nav_msgs::msg::Path> transform(const std::shared_ptr<tf2_ros::Buffer> tf,
                                                    const nav_msgs::msg::Path& path, 
                                                    const std::string& frame,
                                                    const rclcpp::Duration& tf_tolerance)
{
  nav_msgs::msg::Path ret_path;
  ret_path.header.frame_id = frame;
  ret_path.header.stamp = path.header.stamp;
  ret_path.poses.reserve(path.poses.size());

  for(auto& e : path.poses)
  {
    auto t_pose = TfUtility::transform(tf, e, frame, tf_tolerance);
    if(!t_pose)
    {
      //error
      return std::nullopt;
    }
    ret_path.poses.push_back(t_pose.value());
  }

  return ret_path;
}

};




} // namespace wombat


#endif  //TF_UTILITY_H_