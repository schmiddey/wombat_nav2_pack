#ifndef MECANUM_CONTROLLER_H_
#define MECANUM_CONTROLLER_H_

#include "base_controller.hpp"

namespace wombat{


class MecanumController : public BaseController{
public:
  virtual ~MecanumController() = default;
  
  virtual void initialize() override
  {

  }

  virtual geometry_msgs::msg::TwistStamped control(const geometry_msgs::msg::PoseStamped& robot_pose, const nav_msgs::msg::Path& local_path) override
  {

  }

protected:
  MecanumController() = default;
};

} //namespace wombat

#endif  //MECANUM_CONTROLLER_H_