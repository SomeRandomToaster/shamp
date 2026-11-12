cmake_minimum_required(VERSION 3.8)
project(hexa_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)


include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

add_executable(joy_to_cmd src/joy_to_cmd.cpp)
ament_target_dependencies(joy_to_cmd rclcpp sensor_msgs std_msgs)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  TARGETS joy_to_cmd
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
