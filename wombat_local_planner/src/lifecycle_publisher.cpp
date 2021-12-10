#include "wombat_local_planner/lifecycle_publisher.hpp"



namespace wombat
{

LifecylePubHandler::LifecylePubHandler(const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent, const std::string& plugin_name) :
  _node(parent),
  _plugin_name(plugin_name)
{
  if(_node.expired())
  {
    //todo throw exp
    //todo error
  }
  auto node = _node.lock();
  _clock = node->get_clock();
}

nav2_util::CallbackReturn LifecylePubHandler::on_configure() 
{
  auto node = _node.lock();
  if(!node)
  {
    throw std::runtime_error{"Failed to lock Node"};
  }

  // nav2_util::declare_parameter_if_not_declared(node, _plugin_name + "hans", rclcpp::ParameterValue(true));
  // nav2_util::declare_parameter_if_not_declared(node, _plugin_name + "hans", rclcpp::ParameterValue(true));

  _pub_global_path = node->create_publisher<nav_msgs::msg::Path>("given_global_plan", 1);
  _pub_local_path  = node->create_publisher<nav_msgs::msg::Path>("local_plan", 1);
  _pub_dbg_string  = node->create_publisher<std_msgs::msg::String>("wombat_localplanner/debug", 1);

  return nav2_util::CallbackReturn::SUCCESS;
}
nav2_util::CallbackReturn LifecylePubHandler::on_activate() 
{
  _pub_global_path->on_activate();
  _pub_local_path->on_activate();
  _pub_dbg_string->on_activate();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn LifecylePubHandler::on_deactivate() 
{
  _pub_global_path->on_deactivate();
  _pub_local_path->on_deactivate();
  _pub_dbg_string->on_deactivate();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn LifecylePubHandler::on_cleanup() 
{
  _pub_global_path.reset();
  _pub_local_path.reset();
  _pub_dbg_string.reset();

  return nav2_util::CallbackReturn::SUCCESS;
}


void LifecylePubHandler::publish_global_plan(const nav_msgs::msg::Path& path) 
{
  if(!_pub_global_path)
  {
    return;
  }
  _pub_global_path->publish(path);
}

void LifecylePubHandler::publish_local_plan(const nav_msgs::msg::Path& path) 
{
  if(!_pub_local_path)
  {
    return;
  }
  _pub_local_path->publish(path);
}


void LifecylePubHandler::publish_dbg_string(const std::string& msg)
{
  if(!_pub_dbg_string)
  {
    return;
  }
  std_msgs::msg::String str;
  str.data = msg;
  _pub_dbg_string->publish(str);
}




} // namespace wombat
