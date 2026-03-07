cmake_minimum_required(VERSION 3.8)
project(lazy_coulomb_planner)

# ── Compiler flags ─────────────────────────────────────────────────────────────
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)

# ── Dependencies ───────────────────────────────────────────────────────────────
find_package(ament_cmake        REQUIRED)
find_package(rclcpp             REQUIRED)
find_package(rclcpp_lifecycle   REQUIRED)
find_package(nav2_core          REQUIRED)
find_package(nav2_costmap_2d    REQUIRED)
find_package(nav2_util          REQUIRED)
find_package(nav_msgs           REQUIRED)
find_package(geometry_msgs      REQUIRED)
find_package(tf2_ros            REQUIRED)
find_package(pluginlib          REQUIRED)

# ── Shared library ─────────────────────────────────────────────────────────────
add_library(${PROJECT_NAME} SHARED
  src/lazy_coulomb_planner.cpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_lifecycle
  nav2_core
  nav2_costmap_2d
  nav2_util
  nav_msgs
  geometry_msgs
  tf2_ros
  pluginlib
)

# ── Register the plugin with pluginlib ─────────────────────────────────────────
pluginlib_export_plugin_description_file(nav2_core planner_plugins.xml)

# ── Install targets ────────────────────────────────────────────────────────────
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

install(FILES planner_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# ── Export for downstream packages ────────────────────────────────────────────
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  rclcpp
  rclcpp_lifecycle
  nav2_core
  nav2_costmap_2d
  nav2_util
  nav_msgs
  geometry_msgs
  tf2_ros
  pluginlib
)

# ── Tests ──────────────────────────────────────────────────────────────────────
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
