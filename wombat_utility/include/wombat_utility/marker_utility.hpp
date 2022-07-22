#ifndef MARKER_UTILITY_H_
#define MARKER_UTILITY_H_



#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/color_rgba.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace wombat{


class MarkerColor{
public:
  MarkerColor() = delete;
  MarkerColor(const float r, const float g, const float b, const float a = 1.0)
  {
    _color.r = r;
    _color.g = g;
    _color.b = b;
    _color.a = a;
  }

  // { }
  ~MarkerColor() = default;
  
  std_msgs::msg::ColorRGBA toRosColor() const
  {
    return _color;
  }

  static MarkerColor BLACK(float a = 1.0)
  {
    return MarkerColor(0.0, 0.0, 0.0, a);
  }

  static MarkerColor WHITE(float a = 1.0)
  {
    return MarkerColor(1.0, 1.0, 1.0, a);
  }

  static MarkerColor RED(float a = 1.0)
  {
    return MarkerColor(1.0, 0.0, 0.0, a);
  }

  static MarkerColor GREEN(float a = 1.0)
  {
    return MarkerColor(0.0, 1.0, 0.0, a);
  }

  static MarkerColor BLUE(float a = 1.0)
  {
    return MarkerColor(0.0, 0.0, 1.0, a);
  }

  static MarkerColor YELLOW(float a = 1.0)
  {
    return MarkerColor(1.0, 1.0, 0.0, a);
  }

  static MarkerColor CYAN(float a = 1.0)
  {
    return MarkerColor(0.0, 1.0, 1.0, a);
  }

  static MarkerColor MAGENTA(float a = 1.0)
  {
    return MarkerColor(1.0, 0.0, 1.0, a);
  }

  static MarkerColor ORANGE(float a = 1.0)
  {
    return MarkerColor(1.0, 0.5, 0.0, a);
  }


private:
  std_msgs::msg::ColorRGBA _color;
};


/**
 * @brief only static functions for creating marker
 * 
 */
class MarkerUtility{
public:

  inline static visualization_msgs::msg::Marker createSphere(const geometry_msgs::msg::PoseStamped& pose,
                                                             const double scale,
                                                             const MarkerColor& color = MarkerColor::ORANGE(),
                                                             const int id = 0)
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = pose.header.frame_id;
    marker.header.stamp = pose.header.stamp;
    marker.ns = "wombat_utility";
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose = pose.pose;
    marker.scale.x = scale;
    marker.scale.y = scale;
    marker.scale.z = scale;
    marker.color = color.toRosColor();
    // marker.lifetime = //todo
  
   return marker;
  }

  inline static visualization_msgs::msg::Marker createCube(const geometry_msgs::msg::PoseStamped& pose,
                                                          const double scale,
                                                          const MarkerColor& color = MarkerColor::ORANGE(),
                                                          const int id = 0)
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = pose.header.frame_id;
    marker.header.stamp = pose.header.stamp;
    marker.ns = "wombat_utility";
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose = pose.pose;
    marker.scale.x = scale;
    marker.scale.y = scale;
    marker.scale.z = scale;
    marker.color = color.toRosColor();
    // marker.lifetime = //todo
    return marker;
  }


  inline static visualization_msgs::msg::Marker createCylinder(const geometry_msgs::msg::PoseStamped& pose,
                                                              const double height,
                                                              const double diameter,
                                                              const MarkerColor& color = MarkerColor::ORANGE(),
                                                              const int id = 0)
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = pose.header.frame_id;
    marker.header.stamp = pose.header.stamp;
    marker.ns = "wombat_utility";
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::CYLINDER;
    marker.action = visualization_msgs::msg::Marker::ADD;
    
    
    marker.pose.position = pose.pose.position;
    marker.pose.position.z += height * 0.5; //offset by height/2
    marker.pose.orientation.w = 1.0;

    marker.scale.x = diameter;
    marker.scale.y = diameter;
    marker.scale.z = height;
    marker.color = color.toRosColor();
    // marker.lifetime = //todo
    return marker;
  }

  inline static visualization_msgs::msg::Marker createArrow(const geometry_msgs::msg::PoseStamped& pose,
                                                            const double length,
                                                            const double diameter,
                                                            const MarkerColor& color = MarkerColor::ORANGE(),
                                                            const int id = 0)
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = pose.header.frame_id;
    marker.header.stamp = pose.header.stamp;
    marker.ns = "wombat_utility";
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose = pose.pose;
    marker.scale.x = length;
    marker.scale.y = diameter;
    marker.scale.z = diameter;
    marker.color = color.toRosColor();
    // marker.lifetime = //todo
    return marker;
  }


  inline static visualization_msgs::msg::Marker createText(const geometry_msgs::msg::PoseStamped& pose,
                                                           const double scale,
                                                           const std::string& text,
                                                           const double x_offset = 0.0,
                                                            const double y_offset = 0.0,
                                                           const MarkerColor& color = MarkerColor::BLACK(),
                                                           const int id = 0)
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = pose.header.frame_id;
    marker.header.stamp = pose.header.stamp;
    marker.ns = "wombat_utility";
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = scale;
    marker.scale.y = scale;
    marker.scale.z = scale;
    marker.pose = pose.pose;
    marker.pose.position.x += x_offset;
    marker.pose.position.y += y_offset;
    marker.text = text;
    marker.color = color.toRosColor();
    // marker.lifetime = //todo
    return marker;
  }


  inline static visualization_msgs::msg::Marker createLineStrip(std_msgs::msg::Header& header,//const geometry_msgs::msg::PoseStamped& pose,
                                                                const std::vector<geometry_msgs::msg::Point>& points,
                                                                const double scale,
                                                                const MarkerColor& color = MarkerColor::ORANGE(),
                                                                const int id = 0)
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = header.frame_id;
    marker.header.stamp = header.stamp;
    marker.ns = "wombat_utility";
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    // marker.pose = pose.pose; 
    marker.points = points;

    marker.scale.x = scale;

    marker.color = color.toRosColor();
    // marker.lifetime = //todo
    return marker;
  }

  inline static visualization_msgs::msg::Marker createDeleteAll(std_msgs::msg::Header& header,
                                                                const int id = 0)
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = header.frame_id;
    marker.header.stamp = header.stamp;
    marker.ns = "wombat_utility";
    marker.id = id;
    marker.action = visualization_msgs::msg::Marker::DELETEALL;
    return marker;
  }

};

} // namespace wombat

#endif  //MARKER_UTILITY_H_