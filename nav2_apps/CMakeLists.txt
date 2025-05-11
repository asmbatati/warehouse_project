cmake_minimum_required(VERSION 3.8)
project(nav2_apps)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate services
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GoToLoading.srv"
)

install(
  DIRECTORY srv
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
