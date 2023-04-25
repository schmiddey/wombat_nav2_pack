import flet
from flet import UserControl, Page

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from tf2_ros import TransformStamped

from wombat_waypoint.waypoint_repeater_robocup_ros import RepeatRosNode
import wombat_waypoint.waypoint_utils as wp



class ButtonHandler:
  def __init__(self):
    print("creating button handler")

    self.start_pose = None
    self.mid_pose = None
    self.mid_skipped = False
    self.end_pose = None

    self.base_frame = "map"
    self.target_frame = "base_link"

    self.navigating = False

  def to_pose_stamped(self, t:TransformStamped):
    ret = PoseStamped()
    ret.header.frame_id = t.header.frame_id
    ret.header.stamp = t.header.stamp
    ret.pose.position.x = t.transform.translation.x
    ret.pose.position.y = t.transform.translation.y
    ret.pose.position.z = t.transform.translation.z
    ret.pose.orientation.x = t.transform.rotation.x
    ret.pose.orientation.y = t.transform.rotation.y
    ret.pose.orientation.z = t.transform.rotation.z
    ret.pose.orientation.w = t.transform.rotation.w
    return ret

  def set_nav_status(self, status: bool):
    self.navigating = status
    mod = RepeatModel()
    if status:
      mod.btn_start_stop.bgcolor = flet.colors.RED_100
      mod.btn_start_stop.text = "Stop"
      mod.text_toggle.value = "navigating"
      
      mod.page.update()
    else:
      mod.btn_start_stop.bgcolor = flet.colors.GREEN_100
      mod.btn_start_stop.text = "Start"
      mod.text_toggle.value = "stopped"
      mod.page.update()

  def set_cont_state(self, status: bool):
    mod = RepeatModel()
    print(status)
    if status:
      mod.btn_continue.disabled = False
      mod.btn_continue.bgcolor = flet.colors.GREEN_100
    else:
      mod.btn_continue.disabled = True
      mod.btn_continue.bgcolor = flet.colors.RED_100

    pass
  
  def on_set_start(self, e: flet.ControlEvent):
    ros_helper = wp.Ros2NodeHelper()
    t:TransformStamped = ros_helper.get().get_transform(self.base_frame, self.target_frame)
    # t2 = ros_helper.get().get_transform('laser', 'map')
    if t is not None:
      mod = RepeatModel()
      mod.btn_set_start.bgcolor = flet.colors.GREEN_100
      mod.text_start.value = "Start SET"
      mod.page.update()
      self.start_pose = self.to_pose_stamped(t)

  def on_set_mid(self, e: flet.ControlEvent):
    ros_helper = wp.Ros2NodeHelper()
    t:TransformStamped = ros_helper.get().get_transform(self.base_frame, self.target_frame)
    if t is not None:
      mod = RepeatModel()
      mod.btn_set_mid.bgcolor = flet.colors.GREEN_100
      mod.text_mid.value = "Mid SET"
      mod.page.update()
      self.mid_pose = self.to_pose_stamped(t)
      self.mid_skipped = False

  def on_skip_mid(self, e: flet.ControlEvent):
    mod = RepeatModel()
    mod.btn_skip_mid.bgcolor = flet.colors.GREEN_100
    mod.btn_set_mid.disabled = True
    mod.text_mid.value = "Mid SKIPPED"
    mod.page.update()
    self.mid_skipped = True

  def on_set_end(self, e: flet.ControlEvent):
    ros_helper = wp.Ros2NodeHelper()
    t:TransformStamped = ros_helper.get().get_transform(self.base_frame, self.target_frame)
    if t is not None:
      mod = RepeatModel()
      mod.btn_set_end.bgcolor = flet.colors.GREEN_100
      mod.text_end.value = "End SET"
      mod.page.update()
      self.end_pose = self.to_pose_stamped(t)

  def on_start_stop(self, e: flet.ControlEvent):
    ros_helper = wp.Ros2NodeHelper()
    node:RepeatRosNode = ros_helper.get()
    node.start_moving()
    pass

  def on_continue(self, e: flet.ControlEvent):
    ros_helper = wp.Ros2NodeHelper()
    node:RepeatRosNode = ros_helper.get()
    node.continue_nav()
    pass

class RepeatModel():
  _instance = None

  def __new__(cls):
    if cls._instance is None:
      cls._instance = super(RepeatModel, cls).__new__(cls)

      cls.btn_handler = ButtonHandler()
      cls.page = None

      #further init here
      cls.btn_set_start = flet.ElevatedButton(text="set start", on_click=cls.btn_handler.on_set_start, color=flet.colors.ORANGE)
      cls.btn_set_mid = flet.ElevatedButton(text="set mid", on_click=cls.btn_handler.on_set_mid, color=flet.colors.ORANGE)
      cls.btn_skip_mid = flet.ElevatedButton(text="skip mid", on_click=cls.btn_handler.on_skip_mid, color=flet.colors.ORANGE)
      cls.btn_set_end = flet.ElevatedButton(text="set end", on_click=cls.btn_handler.on_set_end, color=flet.colors.ORANGE)
      cls.btn_start_stop = flet.ElevatedButton(text="Start", on_click=cls.btn_handler.on_start_stop, color=flet.colors.ORANGE)
      cls.btn_continue   = flet.ElevatedButton(text="Continue", on_click=cls.btn_handler.on_continue, color=flet.colors.ORANGE)

      #test
      cls.text_start = flet.TextField(label="start", read_only=True, value='txt_start', expand=True, text_size=15, width=100)
      cls.text_mid = flet.TextField(label="mid", read_only=True, value='txt_mid', expand=True, text_size=15, width=100)
      cls.text_end = flet.TextField(label="end", read_only=True, value='txt_end', expand=True, text_size=15, width=100)
      cls.text_toggle = flet.TextField(label="toggle", read_only=True, value='txt_toggle', expand=True, text_size=15, width=100)

      cls.row_width = 400
    return cls._instance

  def set_page(self, page: Page):
    self.page = page

  def create_view(self)-> list[flet.Row]:
    
    return [
      flet.Row(controls=[
        self.text_start,
        self.btn_set_start
    ], width=self.row_width),
    flet.Row(controls=[
        self.text_mid,
        self.btn_set_mid,
        self.btn_skip_mid
    ], width=self.row_width),
    flet.Row(controls=[
        self.text_end,
        self.btn_set_end
    ], width=self.row_width),
    flet.Row(controls=[
        self.text_toggle,
        self.btn_start_stop
    ], width=self.row_width),
    flet.Row(controls=[
        self.btn_continue
    ], width=self.row_width)
    ]