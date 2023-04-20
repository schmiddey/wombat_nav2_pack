#ifndef TF_UTILITY_H_
#define TF_UTILITY_H_

#include<optional>

#include<rclcpp/rclcpp.hpp>
#include<geometry_msgs/msg/pose_stamped.hpp>
#include<nav_msgs/msg/path.hpp>

#include<tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <wombat_utility/wombatPolygon.hpp>

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
template<class T>
static std::optional<T> transform(const std::shared_ptr<tf2_ros::Buffer> tf,
                                                                const T& pose, 
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
      T out_pose;
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

static std::optional<wombat::Polygon2d> transform(const std::shared_ptr<tf2_ros::Buffer> tf, 
                                                  const wombat::Polygon2d& polygon, 
                                                  const std::string frame, 
                                                  const rclcpp::Duration& tf_tolerance)
{
  wombat::Polygon2d ret_polygon;
  auto points = polygon.toRosPoints();
  std::vector<geometry_msgs::msg::Point> ret_points;
  ret_points.reserve(points.size());


  for(auto& e : points)
  { 
    geometry_msgs::msg::PointStamped tmp_point;
    tmp_point.header = polygon.header();
    tmp_point.point = e;
    auto t_point = TfUtility::transform(tf, tmp_point, frame, tf_tolerance);
    
    if(!t_point)
    {
      //error
      return std::nullopt;
    }

    ret_polygon.header() = t_point.value().header;

    ret_points.emplace_back(t_point.value().point);
  }

  ret_polygon.fromRosPoints(ret_points);

  return ret_polygon;
}


static std::optional<geometry_msgs::msg::PoseStamped> lookupTransform(std::shared_ptr<tf2_ros::Buffer> tf, const std::string base_frame, const std::string target_frame)
{
  try
  {
    auto transform = tf->lookupTransform(base_frame, target_frame, tf2::TimePointZero);
    geometry_msgs::msg::PoseStamped p;
    p.header = transform.header;
    p.pose.position.x = transform.transform.translation.x;
    p.pose.position.y = transform.transform.translation.y;
    p.pose.position.z = transform.transform.translation.z;
    p.pose.orientation.x = transform.transform.rotation.x;
    p.pose.orientation.y = transform.transform.rotation.y;
    p.pose.orientation.z = transform.transform.rotation.z;
    p.pose.orientation.w = transform.transform.rotation.w;
    return p;
  }
  catch(tf2::TransformException& e)
  {
    // RCLCPP_ERROR(rclcpp::get_logger("tf_help"),
    //              "Exception in lookupTransform: %s",
    //              e.what()
    //              );
    return std::nullopt;
  }
}



};




} // namespace wombat


#endif  //TF_UTILITY_H_