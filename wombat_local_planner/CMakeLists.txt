cmake_minimum_required(VERSION 3.8)
project(wombat_local_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall  -Wpedantic -Wextra -std=c++17  -Wno-psabi) #
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_common REQUIRED)
find_package(tf2 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_core REQUIRED)
find_package(pluginlib REQUIRED)
find_package(wombat_utility)

nav2_package()

include_directories(
  include
)

set(dependencies
  rclcpp
  std_msgs
  visualization_msgs
  geometry_msgs
  nav_msgs
  nav2_costmap_2d
  pluginlib
  nav_msgs
  nav2_util
  nav2_core
  tf2
  tf2_ros
  nav2_msgs
  wombat_utility
)

add_library(wombat_local_planner SHARED src/wombat_local_planner.cpp
                                        src/lifecycle_publisher.cpp
                                        )


target_compile_definitions(wombat_local_planner PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

ament_target_dependencies(wombat_local_planner
  ${dependencies}
)

install(TARGETS wombat_local_planner
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(wombat_local_planner)
ament_export_dependencies(${dependencies})

pluginlib_export_plugin_description_file(nav2_core wombat_local_planner.xml)

ament_export_include_directories(
  include
)

ament_package()
