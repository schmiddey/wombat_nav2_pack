#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"

#include "wombat_utility/wombat_types.hpp"
#include "wombat_utility/wombatLocalPath2.hpp"

using namespace std::chrono_literals;


class TestWomatUtility : public rclcpp::Node {
public:
  TestWomatUtility() : Node("test_wombat_utils")
  {
    _sub_path = this->create_subscription<nav_msgs::msg::Path>("/plan", 10,
      std::bind(&TestWomatUtility::sub_path_callback, this, std::placeholders::_1));

    _pub_path = this->create_publisher<nav_msgs::msg::Path>("/local_path_hans", 10);

  }
  ~TestWomatUtility()
  {  }

  
private:
  void sub_path_callback(const nav_msgs::msg::Path::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "sub_path_callback");
    
    if(msg->poses.empty())
    {
      RCLCPP_INFO(this->get_logger(), "sub_path_callback: empty path");
      return;
    }

    auto robot_pose = wombat::Pose2(msg->poses.front().pose);

    auto path2 = wombat::Path2(*msg);
    
    _local_path = std::make_unique<wombat::LocalPath2>(path2, 0.2, 1.0);

    _local_path->update(robot_pose);

    auto tmp_local_path = _local_path->extractLocalPath();
  
    std::cout << "tmp_local_path size: " << tmp_local_path.size() << std::endl;

    //pub path
    _pub_path->publish(tmp_local_path.toRosPath());

    RCLCPP_INFO(this->get_logger(), "local_path_length_from_begin: %f", _local_path->localPathLength_from_begin());
  }


private:
  std::unique_ptr<wombat::LocalPath2> _local_path;

  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr _sub_path;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr _pub_path;

};




int main(int argc, char const *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TestWomatUtility>());
  rclcpp::shutdown();

  return 0;
}
