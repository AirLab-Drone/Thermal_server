cmake_minimum_required(VERSION 3.8)
project(check_status)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)


# Install Python modules
ament_python_install_package(${PROJECT_NAME}_py)
# Install Python executables

file(GLOB SCRIPTS "scripts/*.py")
install(PROGRAMS
  ${SCRIPTS}
  scripts/check_thermal_camera.py
  scripts/check_drone_status.py
  scripts/check_UpS_status.py
  scripts/send_fire_alert.py
  DESTINATION lib/${PROJECT_NAME}
)


install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)


ament_package()
