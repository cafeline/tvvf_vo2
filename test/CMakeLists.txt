# test/CMakeLists.txt

# Wavefront Expanderテスト
ament_add_gtest(test_wavefront_expander
    test_wavefront_expander.cpp
)

target_include_directories(test_wavefront_expander PUBLIC
    ${CMAKE_SOURCE_DIR}/include
)

target_link_libraries(test_wavefront_expander
    ${PROJECT_NAME}_core
)

ament_target_dependencies(test_wavefront_expander
    rclcpp
    nav_msgs
    geometry_msgs
    Eigen3
)

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

# Repulsive Forceテスト
ament_add_gtest(test_repulsive_force
    test_repulsive_force.cpp
)

target_include_directories(test_repulsive_force PUBLIC
    ${CMAKE_SOURCE_DIR}/include
)

target_link_libraries(test_repulsive_force
    ${PROJECT_NAME}_core
)

ament_target_dependencies(test_repulsive_force
    rclcpp
    nav_msgs
    geometry_msgs
    Eigen3
)

# Map Repulsionテスト
ament_add_gtest(test_map_repulsion
    test_map_repulsion.cpp
)

target_include_directories(test_map_repulsion PUBLIC
    ${CMAKE_SOURCE_DIR}/include
)

target_link_libraries(test_map_repulsion
    ${PROJECT_NAME}_core
)

ament_target_dependencies(test_map_repulsion
    rclcpp
    nav_msgs
    geometry_msgs
    Eigen3
)

# カバレッジ計測の設定
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --coverage")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} --coverage")
endif()
