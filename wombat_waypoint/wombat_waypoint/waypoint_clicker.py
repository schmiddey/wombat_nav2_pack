###############################################################################
# waypoint_clicker.py
# ros2 node for createing waypoints using Rviz2
#
#
###############################################################################


from dataclasses import dataclass
from dataclasses_json import dataclass_json

from wombat_waypoint import waypoint_utils as wp_utils

import rclpy
from rclpy.node import Node

from std_srvs.srv import Empty

from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from visualization_msgs.msg import MarkerArray





class WaypointClickerNode(Node):
  def __init__(self):
    super().__init__('waypoint_clicker_node')
    self.get_logger().info('WaypointClickerNode started')
    
    #publishers
    self.pub_marker = self.create_publisher(MarkerArray, 'waypoint_marker_array', 10)

    #subscribers
    self.sub_clicked_point     = self.create_subscription(PointStamped, '/clicked_point', self.clickedPoint_callback, 10)
    self.sub_clicked_pose      = self.create_subscription(PoseStamped, '/goal_pose', self.clickedGoalPose_callback, 10)
    self.sub_clicked_init_pose = self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.clickedInitialPose, 10)

    #services
    self.srv_save_wp = self.create_service(Empty, 'save_waypoints', self.saveWaypoints_callback)

    self.current_orientation:Quaternion = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
    self.wp_handler = wp_utils.WaypointHandler()

    #parameter
    self.declare_parameter('waypoint_file', 'waypoints.txt')
    #todo draw direct line between waypoints or use planner_server for generating path between waypoints
    
    self.wp_file = self.get_parameter('waypoint_file').get_parameter_value().string_value
    print('--')
    print(self.wp_file)
    print('--')

    file_ok = self.wp_handler.load(self.wp_file)
    if file_ok:
      print("loaded waypoints from file")
    else:
      print("could not load waypoints from file")

    #pub waypoints
    marker_array = self.wp_handler.toRosMarkerArray(self.get_clock().now().to_msg())
    self.pub_marker.publish(marker_array)

  def save_waypoints(self):
    self.wp_handler.write(self.wp_file)

  def saveWaypoints_callback(self, request, response):
    self.save_waypoints()
    return response

  def clickedPoint_callback(self, msg: PointStamped):
    """topic for adding points to waypoint list

    Args:
        msg (PointStamped): _description_
    """
    self.get_logger().info('clickedPoint_callback')
    #add waypoint
    wp = wp_utils.WaypointHandler.toWaypoint(msg.point, self.current_orientation, msg.header.frame_id, self.wp_handler.size(), "wp_" + str(self.wp_handler.size()))
    wp.z = 0.0
    self.wp_handler.add_waypoint(wp)
    #publish marker
    marker_array = self.wp_handler.toRosMarkerArray(self.get_clock().now().to_msg())
    self.pub_marker.publish(marker_array)
    pass

  def clickedGoalPose_callback(self, msg: PoseStamped):
    """topic for defining the orientation of the following Waypoints

    Args:
        msg (PoseStamped): _description_
    """
    self.get_logger().info('set orientation')
    self.current_orientation = msg.pose.orientation
    # print(msg)
    pass
  
  def clickedInitialPose(self, msg: PoseWithCovarianceStamped):
    """topic for removing the last waypoint

    Args:
        msg (PoseWithCovarianceStamped): _description_
    """
    self.get_logger().info('remove last Waypoint')
    self.wp_handler.remove_last_waypoint()
    #publish marker
    marker_array = self.wp_handler.toRosMarkerArray(self.get_clock().now().to_msg())
    self.pub_marker.publish(marker_array)
    pass






def main(args=None):
  rclpy.init(args=args)

  map_repub_node = WaypointClickerNode()


  try:  # to prevent exception after Ctrl-C // this is still a bug???
    rclpy.spin(map_repub_node)
  except KeyboardInterrupt:
    pass

  print('Saving Waypoints to given file')
  map_repub_node.save_waypoints()

  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  map_repub_node.destroy_node()
  rclpy.shutdown()


if __name__ == '__main__':
  main()