cmake_minimum_required(VERSION 3.16)
project(tvvf_vo_c)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3 -march=native -mtune=native -ffast-math)
endif()

# パフォーマンス最適化の設定
add_compile_definitions(NDEBUG)
set(CMAKE_BUILD_TYPE Release)

# TBB (Threading Building Blocks) for parallel algorithms
find_package(TBB QUIET)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

# Include directories
include_directories(include)

# Core library
add_library(${PROJECT_NAME}_core
  src/core/types.cpp
  src/core/fast_marching.cpp
  src/core/global_field_generator.cpp
  src/core/repulsive_force.cpp
  src/core/map_repulsion.cpp
  src/core/cost_map_builder.cpp
  src/ros/tvvf_vo_node.cpp
  src/ros/tvvf_vo_callbacks.cpp
  src/ros/tvvf_vo_control.cpp
  src/ros/tvvf_vo_control_helpers.cpp
  src/ros/tvvf_vo_visualization.cpp
  src/ros/tvvf_vo_tf.cpp
  src/utils/math_utils.cpp
  src/utils/time_utils.cpp
)

target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}_core
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  visualization_msgs
  tf2_ros
  tf2_geometry_msgs
  Eigen3
)

# Main executable (placeholder for now)
add_executable(${PROJECT_NAME}_node src/tvvf_vo_c.cpp)

target_include_directories(${PROJECT_NAME}_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME}_core)

# Link TBB if available for parallel algorithms
if(TBB_FOUND)
  target_link_libraries(${PROJECT_NAME}_core TBB::tbb)
  target_link_libraries(${PROJECT_NAME}_node TBB::tbb)
  target_compile_definitions(${PROJECT_NAME}_core PRIVATE USE_TBB)
endif()

ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  visualization_msgs
  tf2_ros
  tf2_geometry_msgs
  Eigen3
)

# Install targets
install(TARGETS ${PROJECT_NAME}_core
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install header files
install(DIRECTORY include/
  DESTINATION include/
)

# Install configuration and launch files
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.yaml" PATTERN "*.py"
)

# Export dependencies
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  visualization_msgs
  tf2_ros
  tf2_geometry_msgs
  Eigen3
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  
  # Skip copyright and cpplint for now
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  
  ament_lint_auto_find_test_dependencies()
  
  # Add unit tests
  add_subdirectory(test)
endif()

ament_package()
