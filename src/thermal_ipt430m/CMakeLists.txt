cmake_minimum_required(VERSION 3.8)
project(thermal_ipt430m)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(OpenCV REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)



include_directories(include/thermal_ipt430m)
link_directories(lib)



file(GLOB LIBRARIES_IN_FOLDER "lib/*.so")


foreach(lib ${LIBRARIES_IN_FOLDER})
    message(STATUS "Found library: ${lib}")
endforeach()



# /* ------------------------------ In ros2 node ------------------------------ */
add_executable(thermal_ipt430m_node src/IPT430M_ros2_node.cpp)
target_link_libraries(thermal_ipt430m_node 
    ${LIBRARIES_IN_FOLDER}
    pthread
    m
    udev
)
ament_target_dependencies(thermal_ipt430m_node 
    rclcpp 
    std_msgs 
    sensor_msgs 
    cv_bridge 
    image_transport 
    OpenCV
)

install(TARGETS
  thermal_ipt430m_node
  DESTINATION lib/${PROJECT_NAME}
)


install(DIRECTORY
  launch
  lib
  DESTINATION share/${PROJECT_NAME}/
)



ament_package()
