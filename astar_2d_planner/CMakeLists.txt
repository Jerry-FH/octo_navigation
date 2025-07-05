cmake_minimum_required(VERSION 3.8)
project(astar_2d_planner)

find_package(ament_cmake_ros REQUIRED)
find_package(mbf_octo_core REQUIRED)
find_package(mbf_msgs REQUIRED)
find_package(mbf_utility REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)  # ExePath goal.path
find_package(geometry_msgs REQUIRED)

pluginlib_export_plugin_description_file(mbf_octo_core astar_2d_planner.xml)

# Define library
add_library(${PROJECT_NAME} SHARED
  src/astar_2d_planner.cpp
)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp 
  mbf_octo_core 
  mbf_msgs 
  mbf_utility 
  nav_msgs 
  geometry_msgs
  pluginlib
  sensor_msgs
  pcl_conversions
  tf2_geometry_msgs
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${pluginlib_INCLUDE_DIRS}
)
target_compile_definitions(${PROJECT_NAME} PRIVATE
  "astar_2d_planner_BUILDING_LIBRARY"
)

target_link_libraries(${PROJECT_NAME}
  ${MPI_CXX_LIBRARIES}
)

# Define for exe_path_after_node
# install rule
install(
  TARGETS
    ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib
)

install(
  EXPORT export_${PROJECT_NAME}
  FILE ${PROJECT_NAME}Targets.cmake
  NAMESPACE ${PROJECT_NAME}::
  DESTINATION share/${PROJECT_NAME}/cmake
)

# install plugin XML
install(
  FILES astar_2d_planner.xml
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

# export for ament
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(
  mbf_octo_core
  mbf_msgs
  mbf_utility
  rclcpp
  pluginlib
  nav_msgs
  geometry_msgs
  )
ament_package()
