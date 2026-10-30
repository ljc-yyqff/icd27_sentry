cmake_minimum_required(VERSION 3.10)
project(icd2027sentry_omni_pid_pursuit_controller)

## Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Export compile commands for clangd and clang-tidy
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/omni_pid_pursuit_controller.cpp
  src/pid.cpp
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_tunnel_controller test/test_tunnel_controller.cpp)
  target_link_libraries(test_tunnel_controller ${PROJECT_NAME})
  set(ament_cmake_clang_format_CONFIG_FILE "${CMAKE_SOURCE_DIR}/.clang-format")
  set(ament_cmake_clang_tidy_CONFIG_FILE "${CMAKE_SOURCE_DIR}/.clang-tidy")
  find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif()

pluginlib_export_plugin_description_file(nav2_core icd2027sentry_omni_pid_pursuit_controller.xml)

ament_auto_package(
  USE_SCOPED_HEADER_INSTALL_DIR
)
