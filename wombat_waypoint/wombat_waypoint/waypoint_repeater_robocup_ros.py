from enum import Enum
import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

import copy


import wombat_waypoint.waypoint_utils as wp
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from std_msgs.msg import ColorRGBA

from nav2_msgs.action import NavigateToPose

class NavState(Enum):
  IDLE = 0
  START = 1
  MID = 2
  END = 3

class RepeatRosNode(Node):
  def __init__(self):
    super().__init__('waypoint_repeater_robocup_node')

    # self.nav_to = wp.NavigateToPose(self)
    
    self.tf_l = wp.TfListener(self)

    self.loop_timer = self.create_timer(0.05, self.loop_callback)

    self.pub_markers = self.create_publisher(MarkerArray, 'repeat_markers', 10)
    #pub reverse
    self.pub_rev = self.create_publisher(Bool, '/wombat/set_reverse_mode', 10)

    self.on_loop_callback:callable = None

    self.base_frame = None
    self.target_frame = None

    self.start_pose = None
    self.mid_pose = None
    self.end_pose = None

    self.skip_mid = False

    self.next_pose = None

    self.nav_state = NavState.IDLE
    self.nav_prev_state = NavState.IDLE
    self.rdy = False
    # self.cont_nav = False
    self.wait_mid = False

    #navToPose
    self.nav_to = wp.NavigationExecuter(self)
    self.nav_to.on_fail = self.on_fail
    self.nav_to.on_rdy = self.on_rdy


  def on_fail(self):
    print("on_fial")

  def on_rdy(self):
    print("on_rdy")
    print("nav_state: ", self.nav_state)
    if self.nav_state == NavState.MID:
      print("on_rdy: MID")
      self.wait_mid = True

    self.rdy = True


  def pub_marker(self):
    markers = MarkerArray()
    id = 0
    stamp = self.get_clock().now().to_msg()
    markers.markers.append(wp.MarkerHelper.createDeleteAllMarker(id, stamp, "map"))
    
    colRed = ColorRGBA()
    colRed.r = 1.0
    colRed.g = 0.0
    colRed.b = 0.0
    colRed.a = 1.0
    colTxt = ColorRGBA()
    colTxt.r = 0.0
    colTxt.g = 1.0
    colTxt.b = 1.0
    colTxt.a = 1.0

    if self.start_pose is not None:
      self.start_pose.header.stamp = stamp
      markers.markers.append(wp.MarkerHelper.createCylinderMarker(id, self.start_pose, 0.5, 0.1, colRed))
      id += 1
      markers.markers.append(wp.MarkerHelper.createTextMarker(id, self.start_pose, "Start", 0.2, colTxt))
      id += 1
    if self.mid_pose is not None:
      self.mid_pose.header.stamp = stamp
      markers.markers.append(wp.MarkerHelper.createCylinderMarker(id, self.mid_pose, 0.5, 0.1, colRed))
      id += 1
      markers.markers.append(wp.MarkerHelper.createTextMarker(id, self.mid_pose, "Mid", 0.2, colTxt))
      id += 1
    if self.end_pose is not None:
      self.end_pose.header.stamp = stamp
      markers.markers.append(wp.MarkerHelper.createCylinderMarker(id, self.end_pose, 0.5, 0.1, colRed))
      id += 1
      markers.markers.append(wp.MarkerHelper.createTextMarker(id, self.end_pose, "End", 0.2, colTxt))
      id += 1

    self.pub_markers.publish(markers)


  def pub_reverse(self, reverse):
    msg = Bool()
    msg.data = reverse
    #pub n times
    for i in range(0, 5):
      self.pub_rev.publish(msg)

  
  def loop_callback(self):
    if self.on_loop_callback is not None:
      # print("loop_callback")
      self.on_loop_callback()
    # print("loop")
    #create marker from waypoints and publish them
    self.pub_marker()
    #print rdy
    print("rdy: " + str(self.rdy))
    print("nav_state: " + str(self.nav_state))
    #handle navigation
    if self.nav_state == NavState.IDLE:
      #wait for start
      # print("IDLE")
      pass
    elif self.nav_state == NavState.START:
      if not self.nav_to.navigating and not self.rdy:
        self.nav_to.start(self.start_pose)
      #wait until rdy
      if self.rdy:
        self.rdy = False
        self.pub_reverse(False)
        if self.skip_mid:
          self.nav_state = NavState.END
        else:
          self.nav_state = NavState.MID

        self.nav_prev_state = NavState.START

      pass
    elif self.nav_state == NavState.MID:
      if not self.nav_to.navigating and not self.wait_mid and not self.rdy:
        self.nav_to.start(self.mid_pose)
      #wait until rdy
      if self.rdy:
        #wait for user
        if self.wait_mid:
          return


        self.wait_mid = False

        self.rdy = False
        if self.nav_prev_state == NavState.START:
          self.nav_state = NavState.END
          
        else:
          self.nav_state = NavState.START
        self.nav_prev_state = NavState.MID

      pass
    elif self.nav_state == NavState.END:
      if not self.nav_to.navigating and not self.rdy:
        self.nav_to.start(self.end_pose)
      #wait until rdy
      if self.rdy:
        self.rdy = False
        self.pub_reverse(True)
        if self.skip_mid:
          self.nav_state = NavState.START
        else:
          self.nav_state = NavState.MID

        self.nav_prev_state = NavState.END
    pass


    

  def get_transform(self, target_frame: str, base_brame: str):
    self.target_frame = target_frame
    self.base_frame = base_brame
    return self.tf_l.get_transform(target_frame, base_brame)

  def find_nearest_waypoint(self):
    #get curr pose
    t = self.tf_l.get_transform(self.target_frame, self.base_frame)

    #find nearest waypoint
    dist_start = math.sqrt((t.transform.translation.x - self.start_pose.pose.position.x)**2 + (t.transform.translation.y - self.start_pose.pose.position.y)**2)
    # dist_mid = math.sqrt((t.transform.translation.x - self.mid_pose.pose.position.x)**2 + (t.transform.translation.y - self.mid_pose.pose.position.y)**2)
    dist_end = math.sqrt((t.transform.translation.x - self.end_pose.pose.position.x)**2 + (t.transform.translation.y - self.end_pose.pose.position.y)**2)

    if dist_start < dist_end:
      return NavState.START
    else:
      return NavState.END


  def start_moving(self):
    print("start_moving")
    self.rdy = False
    nearest = self.find_nearest_waypoint()
    print("nearest: ", nearest)
    if nearest == NavState.START:
      self.pub_reverse(False)
    else:
      self.pub_reverse(True)

      
    if self.skip_mid:
      print("skip_mid")
      if nearest == NavState.START:
        self.pub_reverse(False)
        self.nav_state = NavState.END
        print("go to NavState.END")
      else:
        self.nav_state = NavState.START
        self.pub_reverse(True)
        print("go to NavState.START")
    else:
      print("no skip_mid -> so go to mid")
      self.nav_state = NavState.MID
    
    self.nav_prev_state = nearest

  def continue_nav(self):
    print("continue_moving")
    self.wait_mid = False
    
  #   pass
  def is_waiting_at_mid(self):
    return self.wait_mid
  
  def stop_moving(self):
    self.nav_state = NavState.IDLE
    self.nav_prev_state = NavState.IDLE
    
  def is_navigating(self):
    return self.nav_to.navigating

