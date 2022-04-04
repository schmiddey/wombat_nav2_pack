from dataclasses import dataclass
from dataclasses_json import dataclass_json
import copy

import os.path

import random

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
  



