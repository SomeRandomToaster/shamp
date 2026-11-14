cmake_minimum_required(VERSION 3.8)
project(shamp_base)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(shamp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(trajectory_msgs REQUIRED)

add_executable(arthropod_controller_node src/arthropod_controller_node.cpp src/arthropod_controller.cpp)
ament_target_dependencies(arthropod_controller_node
  shamp
  geometry_msgs
  rclcpp
  tf2
  trajectory_msgs
)

install(TARGETS
  arthropod_controller_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
