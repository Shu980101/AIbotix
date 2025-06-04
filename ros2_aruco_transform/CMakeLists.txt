cmake_minimum_required(VERSION 3.8)
project(ros2_aruco_transform)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)

# Generate service
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/TransformService.srv"
  DEPENDENCIES geometry_msgs
)

# Use Python installation
find_package(ament_cmake_python REQUIRED)
# ament_python_install_package(${PROJECT_NAME})


ament_package()