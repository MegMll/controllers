cmake_minimum_required(VERSION 3.8)
project(cartesian_velocity)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Fix Boost MPL template parameter limit when using Pinocchio
# Increase the default limit from 20 to 50 parameters
add_compile_options(-DBOOST_MPL_LIMIT_LIST_SIZE=50)
add_compile_options(-DBOOST_MPL_CFG_NO_PREPROCESSED_HEADERS)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(controller_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(robot_interfaces REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(extender_msgs)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} SHARED
  src/cartesian_velocity_teleop_controller.cpp
)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  controller_interface
  pluginlib
  geometry_msgs
  robot_interfaces
  Eigen3
  extender_msgs
)

# Pluginlib export for controller discovery

# Install the library
install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install the entire config directory (containing controllers.yaml + plugin XML)
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

# Export using pluginlib so the controller_interface can find the custom controller
pluginlib_export_plugin_description_file(
        controller_interface config/cartesian_velocity_controller.xml)

ament_package()