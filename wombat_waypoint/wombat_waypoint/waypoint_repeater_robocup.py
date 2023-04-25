import flet
from flet import UserControl, Page

import copy

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from tf2_ros import TransformStamped

from wombat_waypoint.waypoint_repeater_robocup_ros import RepeatRosNode
import wombat_waypoint.waypoint_utils as wp

from wombat_waypoint.waypoint_repeater_model import RepeatModel, ButtonHandler

def flet_main(page: Page):
  
  page.title = "wombat"
  page.window_width = 450
  page.window_height = 600
  page.update()

  #print window size
  # print(page.window_width)
  # print(page.window_height)
  # page.window_resizable = False
  

  ros_helper: wp.Ros2NodeHelper = wp.Ros2NodeHelper()
  ros_node = RepeatRosNode()
  
  ros_helper.start(ros_node)
  model = RepeatModel()

  def loop_callback():
    ros_node.start_pose = copy.deepcopy(model.btn_handler.start_pose)
    # print("start_pose: ", ros_node.start_pose)
    ros_node.mid_pose =   copy.deepcopy(model.btn_handler.mid_pose)
    # print("mid_pose: ", ros_node.mid_pose)
    ros_node.end_pose =   copy.deepcopy(model.btn_handler.end_pose)
    # print("end_pose: ", ros_node.end_pose)
    ros_node.skip_mid = copy.deepcopy(model.btn_handler.mid_skipped)

    model.btn_handler.set_nav_status(ros_node.is_navigating())

    model.btn_handler.set_cont_state(ros_node.is_waiting_at_mid())





  ros_node.on_loop_callback = loop_callback
  
  model.set_page(page)
  stuff = model.create_view()
  for s in stuff:
    page.add(s)





def main():
  # flet.app(target=main_flet)
  print('start main')
  flet.app(view=flet.FLET_APP , target=flet_main)
  # flet.app(target=main_flet)

if __name__ == '__main__':
  # flet.app(target=main_flet)
  main()