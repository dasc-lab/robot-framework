cmake_minimum_required(VERSION 3.5)
project(dasc_robot)

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

SET(CMAKE_CXX_FLAGS -fPIC)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(px4_ros_com REQUIRED)

add_library(dascBots src/DASCRobots.cpp)
ament_target_dependencies(dascBots rclcpp px4_msgs geometry_msgs tf2 tf2_ros px4_ros_com)
target_include_directories(dascBots PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
ament_export_libraries(dascBots)
install(
  TARGETS dascBots
  EXPORT export_dascBots
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION bin/${PROJECT_NAME}
  INCLUDES DESTINATION include/${PROJECT_NAME}
)

add_library(dasc_robot_lib SHARED src/dasc_robot_lib.cpp)
ament_target_dependencies(dasc_robot_lib rclcpp px4_msgs geometry_msgs tf2 tf2_ros px4_ros_com)
target_link_libraries(dasc_robot_lib dascBots)
target_include_directories(dasc_robot_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
ament_export_libraries(dasc_robot_lib)
install(
  TARGETS dasc_robot_lib
  EXPORT export_dasc_robot_lib
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION bin/${PROJECT_NAME}
  INCLUDES DESTINATION include/${PROJECT_NAME}
)

add_executable(example_control src/example_control.cpp)
ament_target_dependencies(example_control rclcpp px4_msgs geometry_msgs tf2 tf2_ros px4_ros_com) 
target_link_libraries(example_control dascBots)
target_include_directories(example_control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
install(TARGETS
  example_control
  DESTINATION lib/${PROJECT_NAME}
)



add_executable(example_control_multi_rover src/example_control_multi_rover.cpp)
ament_target_dependencies(example_control_multi_rover rclcpp px4_msgs geometry_msgs tf2 tf2_ros px4_ros_com) 
target_link_libraries(example_control_multi_rover dascBots)
target_include_directories(example_control_multi_rover PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
install(TARGETS
  example_control_multi_rover
  DESTINATION lib/${PROJECT_NAME})




add_executable(example_control_geometric src/example_control_geometric.cpp)
ament_target_dependencies(example_control_geometric rclcpp px4_msgs geometry_msgs tf2 tf2_ros px4_ros_com) 
target_link_libraries(example_control_geometric dascBots)
target_include_directories(example_control_geometric PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
install(TARGETS
  example_control_geometric
  DESTINATION lib/${PROJECT_NAME}
)


add_executable(example_control_geometric_velocity src/example_control_geometric_velocity.cpp)
ament_target_dependencies(example_control_geometric_velocity rclcpp px4_msgs geometry_msgs tf2 tf2_ros px4_ros_com) 
target_link_libraries(example_control_geometric_velocity dascBots)
target_include_directories(example_control_geometric_velocity PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
install(TARGETS
  example_control_geometric_velocity
  DESTINATION lib/${PROJECT_NAME}
)


add_executable(example_control_geometric_acceleration src/example_control_geometric_acceleration.cpp)
ament_target_dependencies(example_control_geometric_acceleration rclcpp px4_msgs geometry_msgs tf2 tf2_ros px4_ros_com) 
target_link_libraries(example_control_geometric_acceleration dascBots)
target_include_directories(example_control_geometric_acceleration PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
install(TARGETS
  example_control_geometric_acceleration
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
