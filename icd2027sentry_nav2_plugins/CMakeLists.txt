cmake_minimum_required(VERSION 3.5)
project(icd2027sentry_nav2_plugins)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
add_definitions(-Wall -Werror)

## Export compile commands for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
find_package(yaml_cpp_vendor REQUIRED)
find_package(yaml-cpp REQUIRED)

ament_auto_add_library(icd2027sentry_back_up_free_space_behavior SHARED
  src/behaviors/back_up_free_space.cpp
)

ament_auto_add_library(icd2027sentry_layers SHARED
  src/layers/intensity_voxel_layer.cpp
)
target_link_libraries(icd2027sentry_layers yaml-cpp)

ament_auto_add_library(icd2027sentry_tunnel_navigation_bt_node SHARED
  src/bt/tunnel_navigation.cpp
)
target_compile_definitions(icd2027sentry_tunnel_navigation_bt_node PRIVATE BT_PLUGIN_EXPORT)
target_link_libraries(icd2027sentry_tunnel_navigation_bt_node yaml-cpp)

ament_auto_add_library(icd2027sentry_footprint_grid_planner SHARED
  src/planners/footprint_grid_planner.cpp
)
target_link_libraries(icd2027sentry_footprint_grid_planner yaml-cpp)

ament_auto_add_executable(tunnel_guard src/tunnel_guard.cpp)
target_link_libraries(tunnel_guard yaml-cpp)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_tunnel_clearance tests/test_tunnel_clearance.cpp)
  target_include_directories(test_tunnel_clearance PRIVATE include)
  ament_target_dependencies(test_tunnel_clearance tf2)
  target_link_libraries(test_tunnel_clearance yaml-cpp)
  ament_add_gtest(test_tunnel_bt tests/test_tunnel_bt.cpp TIMEOUT 45)
  ament_target_dependencies(test_tunnel_bt
    nav2_behavior_tree rclcpp geometry_msgs nav_msgs sensor_msgs std_msgs)
  target_compile_definitions(test_tunnel_bt PRIVATE
    TUNNEL_BT_PLUGIN="$<TARGET_FILE:icd2027sentry_tunnel_navigation_bt_node>"
    TUNNEL_BT_TEST_CONFIG="${CMAKE_CURRENT_SOURCE_DIR}/tests/tunnel_bt_test_config.yaml"
    TUNNEL_BT_STAGE_TEST_CONFIG="${CMAKE_CURRENT_SOURCE_DIR}/tests/tunnel_bt_staging_test_config.yaml"
    TUNNEL_BT_TREE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../icd2027sentry_nav_bringup/behavior_trees")
  add_dependencies(test_tunnel_bt icd2027sentry_tunnel_navigation_bt_node)
  ament_add_gtest(test_tunnel_guard tests/test_tunnel_guard.cpp)
  target_include_directories(test_tunnel_guard PRIVATE include)
  ament_target_dependencies(test_tunnel_guard nav2_costmap_2d geometry_msgs)
  target_link_libraries(test_tunnel_guard yaml-cpp)
  ament_add_gtest(test_footprint_grid_planner tests/test_footprint_grid_planner.cpp)
  target_include_directories(test_footprint_grid_planner PRIVATE include)
  ament_target_dependencies(test_footprint_grid_planner nav2_costmap_2d geometry_msgs)
  target_link_libraries(test_footprint_grid_planner yaml-cpp)
  set(ament_cmake_clang_format_CONFIG_FILE "${CMAKE_SOURCE_DIR}/.clang-format")
  set(ament_cmake_clang_tidy_CONFIG_FILE "${CMAKE_SOURCE_DIR}/.clang-tidy")
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

pluginlib_export_plugin_description_file(nav2_core behavior_plugin.xml)
pluginlib_export_plugin_description_file(nav2_core planner_plugins.xml)
pluginlib_export_plugin_description_file(nav2_costmap_2d costmap_plugins.xml)

ament_auto_package(
  USE_SCOPED_HEADER_INSTALL_DIR
)
