cmake_minimum_required(VERSION 3.5)
project(realsense_ros2)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(realsense2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(glfw3 REQUIRED)



find_library(realsense2_LIBRARIES realsense2)
find_path(realsense2_INCLUDE_DIRS librealsense2/rs.hpp)

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${realsense2_INCLUDE_DIRS}
)

## Install launch files.
#install(DIRECTORY
#  launch
#  DESTINATION share/${PROJECT_NAME}/
#)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

## rs_t265_node
add_executable(rs_t265_node src/rs_t265_node.cpp)
ament_target_dependencies(rs_t265_node
  "rclcpp"
  "geometry_msgs"
  "nav_msgs"
  "tf2_msgs"
  "tf2_ros"
  "realsense2" 
)
target_include_directories(rs_t265_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(TARGETS rs_t265_node
  DESTINATION lib/${PROJECT_NAME})

## rs_d435_node
add_executable(rs_d435_node src/rs_d435_node.cpp)
ament_target_dependencies(rs_d435_node
  "rclcpp"
  "geometry_msgs"
  "nav_msgs"
  "realsense2" 
  "glfw3" 

)
target_include_directories(rs_d435_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(TARGETS rs_d435_node
  DESTINATION lib/${PROJECT_NAME})

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
