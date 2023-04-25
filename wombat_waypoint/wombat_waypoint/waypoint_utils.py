from dataclasses import dataclass
from dataclasses_json import dataclass_json
import copy

import os.path
import threading
import random
import sys

import rclpy

from rclpy.action import ActionClient
from rclpy.node import Node

from action_msgs.msg import GoalStatus

import tf2_ros
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


from nav2_msgs.action import NavigateToPose


from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker








class MarkerHelper:
  @staticmethod
  def createArrowMarker(id: int, poseStamped: PoseStamped, length: float, diameter: float, color: ColorRGBA):
    marker = Marker()
    marker.header.frame_id = poseStamped.header.frame_id
    marker.header.stamp = poseStamped.header.stamp
    marker.ns = "waypoint_clicker"
    marker.id = id
    marker.type = Marker.ARROW
    marker.action = Marker.ADD
    marker.pose = copy.deepcopy(poseStamped.pose)
    marker.pose.position.z = 0.0
    marker.scale.x = length
    marker.scale.y = diameter
    marker.scale.z = diameter
    marker.color = color
    return marker

  @staticmethod
  def createSphereMarker(id: int, poseStamped: PoseStamped, scale: float, color: ColorRGBA) -> Marker:
    marker = Marker()
    marker.header.frame_id = poseStamped.header.frame_id
    marker.header.stamp = poseStamped.header.stamp
    marker.ns = "waypoint_clicker"
    marker.id = id
    marker.type = Marker.SPHERE
    marker.action = Marker.ADD
    marker.pose.position = copy.deepcopy(poseStamped.pose.position)
    marker.pose.position.z = 0.0
    marker.pose.orientation.w = 1.0
    marker.scale.x = scale
    marker.scale.y = scale
    marker.scale.z = scale
    marker.color = color
    # marker.lifetime
    return marker


  @staticmethod
  def createCubeMarker(id: int, poseStamped: PoseStamped, scale: float, color: ColorRGBA) -> Marker:
    marker = Marker()
    marker.header.frame_id = poseStamped.header.frame_id
    marker.header.stamp = poseStamped.header.stamp
    marker.ns = "waypoint_clicker"
    marker.id = id
    marker.type = Marker.CUBE
    marker.action = Marker.ADD
    marker.pose = copy.deepcopy(poseStamped.pose)
    marker.pose.position.z = 0.0
    marker.scale.x = scale
    marker.scale.y = scale
    marker.scale.z = scale
    marker.color = color
    # marker.lifetime
    return marker

  @staticmethod
  def createCylinderMarker(id: int, poseStamped: PoseStamped, height: float, diameter: float, color: ColorRGBA):
    marker = Marker()
    marker.header.frame_id = poseStamped.header.frame_id
    marker.header.stamp = poseStamped.header.stamp
    marker.ns = "waypoint_clicker"
    marker.id = id
    marker.type = Marker.CYLINDER
    marker.action = Marker.ADD
    marker.pose = copy.deepcopy(poseStamped.pose)
    marker.pose.position.z = 0.25
    marker.scale.x = diameter
    marker.scale.y = diameter
    marker.scale.z = height
    marker.color = color
    # marker.lifetime
    return marker

  @staticmethod
  def createTextMarker(id: int, poseStamped: PoseStamped, text: str, scale: float, color: ColorRGBA):
    marker = Marker()
    marker.header.frame_id = poseStamped.header.frame_id
    marker.header.stamp = poseStamped.header.stamp
    marker.ns = "waypoint_clicker"
    marker.id = id
    marker.type = Marker.TEXT_VIEW_FACING
    marker.action = Marker.ADD
    marker.pose = copy.deepcopy(poseStamped.pose)
    marker.pose.position.z = 0.5
    marker.scale.x = scale
    marker.scale.y = scale
    marker.scale.z = scale
    marker.color = color
    marker.text = text
    # marker.lifetime
    return marker

  @staticmethod
  def createLineStripMarker(id: int, stamp, frame_id, width: float, points, color: ColorRGBA):
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = stamp
    marker.ns = "waypoint_clicker"
    marker.id = id
    marker.type = Marker.LINE_STRIP
    marker.action = Marker.ADD
    # marker.pose = copy.deepcopy(poseStamped.pose)
    # marker.pose.position.z = 0.0
    marker.pose.orientation.w = 1.0
    marker.scale.x = width
    marker.scale.y = width
    marker.scale.z = width
    marker.color = color
    marker.points = points
    # marker.lifetime
    return marker

  @staticmethod
  def createDeleteAllMarker(id: int, stamp, frame_id = "map"):
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = stamp
    marker.ns = "waypoint_clicker"
    marker.id = id
    marker.type = Marker.CUBE
    marker.action = Marker.DELETEALL
    return marker





@dataclass_json
@dataclass
class Waypoint:
  x: float
  y: float
  z: float
  qx: float
  qy: float
  qz: float
  qw: float
  frame_id: str
  id: int
  name: str
  #todo more ggf service name or other stuff







#class for Handling Waypoints and serialize and deserialize Waypoints to file
class WaypointHandler:
  def __init__(self):
    self.waypoints = []

  def shuffle(self):
    random.shuffle(self.waypoints)

  def add_waypoint(self, waypoint: Waypoint):
    self.waypoints.append(waypoint)

  def remove_last_waypoint(self):
    self.waypoints.pop(-1)

  def get_waypoints(self):
    return self.waypoints

  def write(self, filename: str):
    print("Writing ", self.size(), " waypoints to ", filename)
    file = open(filename, "w")
    for waypoint in self.waypoints:
      line = waypoint.to_json() + "\n"
      file.write(line)
    file.close()
    print("Writing done")
    pass

  def load(self, filename: str):
    print("Loading waypoints from ", filename)
    #check if file exists
    if not os.path.isfile(filename):
      print("File does not exist")
      return False


    self.waypoints.clear()
    file = open(filename, "r")
    lines = file.readlines()
    for line in lines:
      self.waypoints.append(Waypoint.from_json(line))
    file.close()
    print("Loading done, loaded  ", self.size(), " waypoints")
    return True

  def print(self):
    for waypoint in self.waypoints:
      print(waypoint)

  def size(self):
    return len(self.waypoints)

  def empty(self):
    return self.size() == 0

  def toWaypoint(pos: Point, ori: Quaternion, frame_id: str,  id: int, name: str):
    waypoint = Waypoint(pos.x, pos.y, pos.z,
                        ori.x, ori.y, ori.z, ori.w, 
                        frame_id, id, name)
    return waypoint

  def toRosPose(waypoint: Waypoint, stamp):
    pose = PoseStamped()
    pose.header.frame_id = waypoint.frame_id
    pose.header.stamp = stamp
    pose.pose.position.x = waypoint.x
    pose.pose.position.y = waypoint.y
    pose.pose.position.z = waypoint.z
    pose.pose.orientation.x = waypoint.qx
    pose.pose.orientation.y = waypoint.qy
    pose.pose.orientation.z = waypoint.qz
    pose.pose.orientation.w = waypoint.qw
    return pose

  def toRosPoseArray(self, stamp):
    poseArray = []
    for waypoint in self.waypoints:
      poseArray.append(WaypointHandler.toRosPose(waypoint, stamp))
    return poseArray

  def toRosMarkerArray(self, stamp):
    markers: MarkerArray = MarkerArray()
    line_points = []
    id: int = 0
    markers.markers.append(MarkerHelper.createDeleteAllMarker(id, stamp))
    id += 1
    point_old = Point()
    first_loop = True

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

    for wp in self.waypoints:
      pose = WaypointHandler.toRosPose(wp, stamp)

      if(not first_loop):
        line_points.append(copy.deepcopy(point_old))
        line_points.append(copy.deepcopy(pose.pose.position))
      point_old = copy.deepcopy(pose.pose.position)
      first_loop = False
      
      marker_txt = MarkerHelper.createTextMarker(id, pose, wp.name, 0.2, colTxt)
      id += 1
      marker = MarkerHelper.createSphereMarker(id, pose, 0.2, colRed)
      id += 1
      marker_cyl = MarkerHelper.createCylinderMarker(id, pose, 0.4, 0.1, colRed)
      id += 1
      marker_arrow = MarkerHelper.createArrowMarker(id, pose, 0.4, 0.03, colRed)
      id += 1
      markers.markers.append(marker_txt)
      markers.markers.append(marker)
      markers.markers.append(marker_cyl)
      markers.markers.append(marker_arrow)
    
    #create line marker if not empty
    if not self.empty():
      marker_line = MarkerHelper.createLineStripMarker(id, stamp, "map", 0.05, line_points, colRed)
      markers.markers.append(marker_line)

    return markers
  

#todo singleton


# class Logger:
#   _instance = None

#   def __new__(cls):
#     if cls._instance is None:
#       print('Creating the object')
#       cls._instance = super(Logger, cls).__new__(cls)
#       # Put any initialization here.
#     return cls._instance
class Ros2NodeHelper:
  _instance = None

  def __new__(cls):
    if cls._instance is None:
      cls._instance = super(Ros2NodeHelper, cls).__new__(cls)
      cls.node: Node = None
      cls.running = False
      print(sys.argv)
      rclpy.init(args=sys.argv)
    return cls._instance

  def get(self) ->Node:
    return self.node

  def start(self, node: Node):
    if self.running:
      return
        
    self.node = node


    #start thread
    self.thrd = threading.Thread(target=self._spinner, args=(), daemon=True)
    self.running = True
    self.thrd.start()

  def _spinner(self):

    try:  # to prevent exception after Ctrl-C // this is still a bug???
      rclpy.spin(self.node)
    except KeyboardInterrupt:
      pass
    print("spinner stopped")
    self.node.destroy_node()
    rclpy.shutdown()
    self.running = False
    

  def stop(self):
    rclpy.shutdown()





class NavigationExecuter():
  def __init__(self, node: Node):
    self.node = node
    self.action_client = ActionClient(node, NavigateToPose, "/navigate_to_pose")
    self.on_feedback:callable = None
    self.on_response:callable = None
    self.on_rdy:callable = None

    self.on_fail:callable = None

    self.navigating = False
    

  def start(self, pose: PoseStamped):
    self.navigating = True
    goal_msg = NavigateToPose.Goal()
    goal_msg.pose = pose
    self.action_client.wait_for_server(2.0)
    self.send_future = self.action_client.send_goal_async(goal_msg, 
                                feedback_callback=self._feedback_callback
                                # result_callback=self._get_result_callback
                                )
    #add response callback
    self.send_future.add_done_callback(self._goal_response_callback)

  def _goal_response_callback(self, future):
    goal_handle = future.result()
    if not goal_handle.accepted:
      print('Goal rejected :(')
      if self.on_fail is not None:
        self.on_fail()
        self.navigating = False
      return

    self._get_result_future = goal_handle.get_result_async()
    self._get_result_future.add_done_callback(self._get_result_callback)

    print('Goal accepted :)')

    # self.get_result(goal_handle)

  def _get_result_callback(self, future):
    result = future.result().result
    status = future.result().status
    self.navigating = False
    if status == GoalStatus.STATUS_SUCCEEDED:
        self.node.get_logger().info('Goal succeeded! Result')
        if self.on_rdy is not None:
          self.on_rdy()
    else:
        self.node.get_logger().info('Goal failed with status: {0}'.format(status))
        if self.on_fail is not None:
          self.on_fail()

  def _feedback_callback(self, feedback_msg):
    # print('Received feedback: {0}'.format(feedback_msg.feedback))
    feedback: NavigateToPose.Feedback = feedback_msg.feedback
    if self.on_feedback is not None:
      self.on_feedback(feedback)

    
    


class TfListener():
  def __init__(self, node: Node):
    self.node = node

    self.tf_buffer = Buffer()
    self.tf_listener = TransformListener(self.tf_buffer, self.node)


  def get_transform(self, target_frame:str, base_frame:str):
    try:
      t = self.tf_buffer.lookup_transform(
                              target_frame,
                              base_frame,
                              rclpy.time.Time()
                              # self.node.get_clock().now(),
                              # rclpy.duration.Duration(seconds=1.0)
                              )
      return t
    except TransformException as ex:
      self.node.get_logger().info(
        f'Could not transform {base_frame} to {target_frame}: {ex}')
      return None