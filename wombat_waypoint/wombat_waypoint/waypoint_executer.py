from distutils.log import ERROR
from enum import Enum
import rclpy

from rclpy.action import ActionClient
from rclpy.node import Node

from wombat_waypoint import waypoint_utils as wp_utils

from nav2_msgs.action import FollowWaypoints
from std_srvs.srv import Empty
from visualization_msgs.msg import MarkerArray

class ExecuterState(Enum):
  IDLE = 0
  FOLLOWING_WAYPOINTS = 1
  READY = 2
  ERROR = 3

class WaypointExecuterNode(Node):
  def __init__(self):
    super().__init__('waypoint_executer_node')
    self.get_logger().info('WaypointExecuterNode started')
    self.wp_handler = wp_utils.WaypointHandler()

    self.action_client = ActionClient(self, FollowWaypoints, 'follow_waypoints')
    
    self.state = ExecuterState.IDLE

    # self.stop_following = False
    
    #parameter
    self.declare_parameter('waypoint_file', 'waypoints.txt')
    self.declare_parameter('endless_loop', True)
    self.declare_parameter('autostart', True)
    self.declare_parameter('shuffle_when_rdy', True)

    self.wp_file      = self.get_parameter('waypoint_file').get_parameter_value().string_value
    self.endless_loop = self.get_parameter('endless_loop').get_parameter_value().bool_value
    self.autostart    = self.get_parameter('autostart').get_parameter_value().bool_value
    self.shuffle_when_rdy = self.get_parameter('shuffle_when_rdy').get_parameter_value().bool_value

    #load waypoints
    file_ok = self.wp_handler.load(self.wp_file)
    if not file_ok:
      self.get_logger().info("could not load waypoints from file... will exit")
      rclpy.shutdown()
      exit(1)

    #publishers
    self.pub_marker = self.create_publisher(MarkerArray, 'waypoint_marker_array', 10)

    #service
    #empty service for restarting waypoints
    self.srv_restart = self.create_service(Empty, 'restart_waypoints', self.srv_restart_waypoints_callback)

    #timer
    self.timer = self.create_timer(0.2, self.timer_callback)


  def srv_restart_waypoints_callback(self, req, resp: Empty.Response):
    # print("srv called")
    # self.send_waypoints()
    if self.state == ExecuterState.IDLE or self.state == ExecuterState.READY:
      self.send_waypoints()
    return resp

  def timer_callback(self):
    # self.send_waypoints()
    # print("timer called")
    if self.state == ExecuterState.IDLE:
      if self.autostart:
        self.send_waypoints()
      pass
    
    if self.state == ExecuterState.READY:
      if self.endless_loop:
        
        self.send_waypoints()
      pass
    
    # if self.state == ExecuterState.FOLLOWING_WAYPOINTS:
    #   if self.stop_following:
    #     self.cancel_future = self.action_client._cancel_goal_async(self.goal_handle)
    #     pass
    #   pass

    # pass
  
  def publish_markers(self):
    marker_array = MarkerArray()
    marker_array = self.wp_handler.toRosMarkerArray(self.get_clock().now().to_msg())
    self.pub_marker.publish(marker_array)
    pass

  def stop(self):
    # self.stop_following = True
    #not a good solution, but works for now
    self.action_client._cancel_goal_async(self.goal_handle)
    pass

  def send_waypoints(self):
    if self.shuffle_when_rdy:
      self.wp_handler.shuffle()
    self.publish_markers()

    goal_msg = FollowWaypoints.Goal()
    goal_msg.poses = self.wp_handler.toRosPoseArray(self.get_clock().now().to_msg())
    #todo check if this is better somewhere esle
    self.action_client.wait_for_server()

    self.send_goal_future = self.action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
    self.send_goal_future.add_done_callback(self.goal_response_callback)
    
    self.state = ExecuterState.FOLLOWING_WAYPOINTS
    pass

  def goal_response_callback(self, future):
    self.goal_handle = future.result()
    if not self.goal_handle.accepted:
      self.get_logger().info('Goal rejected')
      self.state = ExecuterState.ERROR
      return

    self.get_logger().info('Goal accepted')
    self.get_result_future = self.goal_handle.get_result_async()
    self.get_result_future.add_done_callback(self.get_result_callback)

  def get_result_callback(self, future):
    result = future.result().result
    print("Result, missed waypoints: ", result.missed_waypoints)
    self.state = ExecuterState.READY


  def feedback_callback(self, feedback_msg):
    self.get_logger().info('Feedback received')
    print("Current Waypoint: ", feedback_msg.feedback.current_waypoint)


def main(args=None):
  rclpy.init(args=args)
  node = WaypointExecuterNode()
  
  try:
    rclpy.spin(node)
  except KeyboardInterrupt:
    pass

  print("hans")
  node.stop()

  node.destroy_node()
  rclpy.shutdown()