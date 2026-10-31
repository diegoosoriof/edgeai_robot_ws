cmake_minimum_required(VERSION 3.8)
project(edgeai_robot_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc flann)

add_executable(map_alignment_node src/map_alignment_node.cpp)
target_compile_features(map_alignment_node PUBLIC cxx_std_17)
target_include_directories(map_alignment_node PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(map_alignment_node ${OpenCV_LIBRARIES})
ament_target_dependencies(map_alignment_node
  rclcpp nav_msgs geometry_msgs std_msgs std_srvs visualization_msgs
  tf2 tf2_ros tf2_geometry_msgs)
install(TARGETS map_alignment_node DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY
    launch
    urdf
    worlds
    rviz
    config
    maps
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/teleop.py
  DESTINATION lib/${PROJECT_NAME}
)

#if(BUILD_TESTING)
#  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
#  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
#  # a copyright and license is added to all source files
#  set(ament_cmake_cpplint_FOUND TRUE)
#  ament_lint_auto_find_test_dependencies()
#endif()

ament_package()
