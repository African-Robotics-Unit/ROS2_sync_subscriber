cmake_minimum_required(VERSION 3.5)
project(sync_subscriber)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(flir_boson_interfaces REQUIRED)


# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(camera_sync_node src/camera_sync_node.cpp)
ament_target_dependencies(camera_sync_node 
"rclcpp"
"sensor_msgs"
"std_msgs"
"cv_bridge"
"message_filters"
)

add_executable(camera_sync_node_lepton src/camera_sync_node_lepton.cpp)
ament_target_dependencies(camera_sync_node_lepton
"rclcpp"
"sensor_msgs"
"std_msgs"
"cv_bridge"
"message_filters"
)

target_include_directories(camera_sync_node PUBLIC
  ${OpenCV_INCLUDE_DIRS}
)
target_include_directories(camera_sync_node_lepton PUBLIC
  ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(camera_sync_node
  ${OpenCV_LIBRARIES}
)
target_link_libraries(camera_sync_node_lepton
  ${OpenCV_LIBRARIES}
)

install(TARGETS
  camera_sync_node
  camera_sync_node_lepton
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
