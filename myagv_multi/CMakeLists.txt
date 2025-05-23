cmake_minimum_required(VERSION 3.8)
project(myagv_multi)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(myagv_odometry REQUIRED)
find_package(myagv_navigation2 REQUIRED)
find_package(rosidl_default_generators REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Position.msg"
 )

#add_executable(my_node src/my_node.cpp)
#target_include_directories(my_node PUBLIC
#  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#  $<INSTALL_INTERFACE:include>)
#target_compile_features(my_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

#install(TARGETS my_node
#  DESTINATION lib/${PROJECT_NAME})

include_directories(
  include
)

add_executable(multi_slave src/slave_tf_listener.cpp)
ament_target_dependencies(multi_slave tf2_ros tf2 tf2_geometry_msgs geometry_msgs rclcpp std_msgs nav_msgs std_srvs sensor_msgs)
install(TARGETS multi_slave DESTINATION  lib/${PROJECT_NAME})

add_executable(multi_avoidance src/multi_avoidance.cpp)
ament_target_dependencies(multi_avoidance tf2_ros tf2 tf2_geometry_msgs rclcpp geometry_msgs std_msgs nav_msgs std_srvs sensor_msgs)
install(TARGETS multi_avoidance DESTINATION  lib/${PROJECT_NAME})

rosidl_target_interfaces(multi_avoidance ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS scripts/send_tfodom.py scripts/listen_tfodom.py scripts/set_pose.py scripts/lasertracker.py DESTINATION lib/${PROJECT_NAME})

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

ament_package()
