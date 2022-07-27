#ifndef LIMIT_ACCEL_H_
#define LIMIT_ACCEL_H_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>



namespace wombat {

class LimitAccel {
public:
  LimitAccel(rclcpp::Clock::SharedPtr clock, const double max_accel_linear, const double max_accel_angular) :
    _max_accel_linear(max_accel_linear), 
    _max_accel_angular(max_accel_angular),
    _clock(clock)
  {
    _last_twist.angular.x = 0;
    _last_twist.angular.y = 0;
    _last_twist.angular.z = 0;
    _last_twist.linear.x = 0;
    _last_twist.linear.y = 0;
    _last_twist.linear.z = 0;
    
    _time_last_twist = _clock->now();
  }
  ~LimitAccel() = default;

  geometry_msgs::msg::Twist limitAccel(const geometry_msgs::msg::Twist& twist)
  {
    geometry_msgs::msg::Twist limited_twist;

    auto delta_lin_x = twist.linear.x - _last_twist.linear.x;
    auto delta_lin_y = twist.linear.y - _last_twist.linear.y;
    // auto delta_lin_z = twist.linear.z - _last_twist.linear.z;
    // auto delta_ang_x = twist.angular.x - _last_twist.angular.x;
    // auto delta_ang_y = twist.angular.y - _last_twist.angular.y;
    auto delta_ang_z = twist.angular.z - _last_twist.angular.z;

    auto delta_time = (_clock->now() - _time_last_twist).seconds();

    auto dir_lin_x = delta_lin_x > 0 ? 1.0 : -1.0;
    auto dir_lin_y = delta_lin_y > 0 ? 1.0 : -1.0;
    auto dir_ang   = delta_ang_z > 0 ? 1.0 : -1.0;

    
    limited_twist.linear.x  = (std::abs(delta_lin_x) > _max_accel_linear * delta_time)  ? _last_twist.linear.x  + _max_accel_linear  * delta_time * dir_lin_x : twist.linear.x;
    limited_twist.linear.y  = (std::abs(delta_lin_y) > _max_accel_linear * delta_time)  ? _last_twist.linear.y  + _max_accel_linear  * delta_time * dir_lin_y : twist.linear.y;
    limited_twist.angular.z = (std::abs(delta_ang_z) > _max_accel_angular * delta_time) ? _last_twist.angular.z + _max_accel_angular * delta_time * dir_ang   : twist.angular.z;


    _last_twist = limited_twist;
    _time_last_twist = _clock->now();

    return limited_twist;
  }
  
private:

  double _max_accel_linear;  // delta velocity per second
  double _max_accel_angular; // delta velocity per second

  geometry_msgs::msg::Twist _last_twist;

  rclcpp::Time _time_last_twist;

  rclcpp::Clock::SharedPtr _clock;
  
};



} // namespace wombat



#endif  //LIMIT_ACCEL_H_