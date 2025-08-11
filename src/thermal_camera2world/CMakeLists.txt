cmake_minimum_required(VERSION 3.8)
project(thermal_camera2world)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# /* ---------------------------- Find dependencies --------------------------- */
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(thermal_msgs REQUIRED)


# /* --------------------- Include Cpp "include" directory -------------------- */
include_directories(include)


# /* -------------------------- Create Cpp executable ------------------------- */
# add_executable(cpp_executable src/cpp_node.cpp)

# ament_target_dependencies(cpp_executable rclcpp)

# install(TARGETS
#   cpp_executable
#   DESTINATION lib/${PROJECT_NAME}
# )


# /* ------------------------- Install Python modules ------------------------- */
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/thermal_camera2world.py
  scripts/compare_thermalAlert.py
  scripts/setup_camera2world_point.py
  scripts/read_robot_laser_back.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)



ament_package()
