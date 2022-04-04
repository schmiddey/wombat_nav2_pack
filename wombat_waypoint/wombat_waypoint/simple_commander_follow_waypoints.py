
from wombat_waypoint import waypoint_utils as wp_utils

from nav2_simple_commander.robot_navigator import BasicNavigator, NavigationResult

import rclpy


rclpy.init()
nav = BasicNavigator()

#params
nav.declare_parameter('waypoint_file', 'waypoints.txt')
nav.declare_parameter('endless_loop', True)

wp_file = nav.get_parameter('waypoint_file').get_parameter_value().string_value
do_endless_loop = nav.get_parameter('endless_loop').get_parameter_value().bool_value

print('-- Parameters --')
print('waypointfile: ', wp_file)
print('endless loop: ', do_endless_loop)
print('-- --')

wp_handler = wp_utils.WaypointHandler()

file_ok = wp_handler.load(wp_file)

if not file_ok:
  print("could not load waypoints from file... will exit")
  rclpy.shutdown()
  exit(1)


# nav.waitUntilNav2Active()

while rclpy.ok() and do_endless_loop:
  #folow waypoints
  stamp = nav.get_clock().now().to_msg()
  nav.followWaypoints(wp_handler.toRosPoseArray(stamp))
  
  
  while not nav.isNavComplete():
    feedback = nav.getFeedback()


result = nav.getResult()
if result ==   NavigationResult.SUCCEEDED:
    print('Goal succeeded!')
elif result == NavigationResult.CANCELED:
    print('Goal was canceled!')
elif result == NavigationResult.FAILED:
    print('Goal failed!')


