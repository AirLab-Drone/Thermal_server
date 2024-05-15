cmake_minimum_required(VERSION 3.8)
project(thermal_im4)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(thermal_msgs REQUIRED)
find_package(std_srvs REQUIRED)


include_directories(include/thermal_im4)
link_directories(lib)



file(GLOB LIBRARIES_IN_FOLDER "lib/*.so")


foreach(lib ${LIBRARIES_IN_FOLDER})
    message(STATUS "Found library: ${lib}")
endforeach()



# /* ------------------------------ ros2 node ------------------------------ */
# add_executable(test_thermal_im4_node src/test_thermal_im4_node.cpp)
# target_link_libraries(test_thermal_im4_node 
#     ${LIBRARIES_IN_FOLDER}
#     pthread
#     m
#     udev
# )
# ament_target_dependencies(test_thermal_im4_node 
#     rclcpp 
#     std_msgs 
#     sensor_msgs 
#     cv_bridge 
#     image_transport 
#     OpenCV
#     thermal_msgs
#     std_srvs
# )

# /* ---------------------------------- test ---------------------------------- */
add_executable(test_im4 src/test_thermal_im4_node.cpp)
target_link_libraries(test_im4 
    ${OpenCV_LIBS}
    ${LIBRARIES_IN_FOLDER}
    pthread
    m
    udev
)



install(TARGETS
  # test_thermal_im4_node
  DESTINATION lib/${PROJECT_NAME}
)


install(DIRECTORY
  launch
  lib
  DESTINATION share/${PROJECT_NAME}/
)



ament_package()

