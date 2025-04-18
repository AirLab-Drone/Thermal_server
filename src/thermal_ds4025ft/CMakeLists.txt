cmake_minimum_required(VERSION 3.8)
project(thermal_ds4025ft)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# /* ---------------------------- Find dependencies --------------------------- */
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)


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
  # scripts/test_thermal_ds4025ft.py
  scripts/DS4025FT_ros2_node.py
  # scripts/test_code.py
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)
ament_package()
