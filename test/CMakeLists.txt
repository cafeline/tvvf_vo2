# test/CMakeLists.txt

# Fast Marchingテスト
ament_add_gtest(test_fast_marching
    test_fast_marching.cpp
)

target_include_directories(test_fast_marching PUBLIC
    ${CMAKE_SOURCE_DIR}/include
)

target_link_libraries(test_fast_marching
    ${PROJECT_NAME}_core
)

ament_target_dependencies(test_fast_marching
    rclcpp
    nav_msgs
    geometry_msgs
    Eigen3
)

# Smooth Velocity Optimizerテスト
ament_add_gtest(test_smooth_velocity_optimizer
    test_smooth_velocity_optimizer.cpp
)

target_include_directories(test_smooth_velocity_optimizer PUBLIC
    ${CMAKE_SOURCE_DIR}/include
)

target_link_libraries(test_smooth_velocity_optimizer
    ${PROJECT_NAME}_core
)

ament_target_dependencies(test_smooth_velocity_optimizer
    rclcpp
    nav_msgs
    geometry_msgs
    Eigen3
)

# Cost Map Builderテスト
ament_add_gtest(test_cost_map_builder
    test_cost_map_builder.cpp
)

target_include_directories(test_cost_map_builder PUBLIC
    ${CMAKE_SOURCE_DIR}/include
)

target_link_libraries(test_cost_map_builder
    ${PROJECT_NAME}_core
)

ament_target_dependencies(test_cost_map_builder
    rclcpp
    nav_msgs
    geometry_msgs
    Eigen3
)

# Cost Map Visualizationテスト
ament_add_gtest(test_costmap_visualization
    test_costmap_visualization.cpp
)

target_include_directories(test_costmap_visualization PUBLIC
    ${CMAKE_SOURCE_DIR}/include
)

target_link_libraries(test_costmap_visualization
    ${PROJECT_NAME}_core
)

ament_target_dependencies(test_costmap_visualization
    rclcpp
    nav_msgs
    geometry_msgs
    Eigen3
)

# Global Field Generatorテスト
ament_add_gtest(test_global_field_generator
    test_global_field_generator.cpp
)

target_include_directories(test_global_field_generator PUBLIC
    ${CMAKE_SOURCE_DIR}/include
)

target_link_libraries(test_global_field_generator
    ${PROJECT_NAME}_core
)

ament_target_dependencies(test_global_field_generator
    rclcpp
    nav_msgs
    geometry_msgs
    Eigen3
)

# Unified obstacles test
ament_add_gtest(test_obstacles_unified
    test_obstacles_unified.cpp
)

target_include_directories(test_obstacles_unified PUBLIC
    ${CMAKE_SOURCE_DIR}/include
)

target_link_libraries(test_obstacles_unified
    ${PROJECT_NAME}_core
)

ament_target_dependencies(test_obstacles_unified
    rclcpp
    nav_msgs
    geometry_msgs
    Eigen3
)

# Control loop safety (stop on missing TF/goal)
ament_add_gtest(test_control_loop_safety
    test_control_loop_safety.cpp
)

target_include_directories(test_control_loop_safety PUBLIC
    ${CMAKE_SOURCE_DIR}/include
)

target_link_libraries(test_control_loop_safety
    ${PROJECT_NAME}_core
)

ament_target_dependencies(test_control_loop_safety
    rclcpp
    nav_msgs
    geometry_msgs
    tf2_ros
    tf2_geometry_msgs
    Eigen3
)

# カバレッジ計測の設定
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --coverage")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} --coverage")
endif()
