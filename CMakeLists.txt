cmake_minimum_required(VERSION 3.8)
project(sicks300_ros2_scan_merger)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

################################################
## Find  dependencies                         ##
################################################
## Find ament macros and libraries
find_package(ament_cmake REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rcutils REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(PCL REQUIRED)


###########
## Build ##
###########
## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
)
add_executable(${PROJECT_NAME}
  src/common/ScannerSickS300.cpp
  src/common/SerialIO.cpp
  src/sicks300_node.cpp
  src/main.cpp
)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp_lifecycle
  std_msgs
  sensor_msgs
  diagnostic_msgs
  nav2_util
  rclcpp 
  pcl_conversions
  rcl_interfaces
  rcutils
  tf2_geometry_msgs
  geometry_msgs
)
target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})

install(TARGETS
${PROJECT_NAME}
DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.hpp"
#   PATTERN ".svn" EXCLUDE
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY params
  DESTINATION share/${PROJECT_NAME}
)


#############
## Testing ##
#############
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()


ament_package()
