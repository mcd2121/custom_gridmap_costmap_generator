cmake_minimum_required(VERSION 3.8)
project(custom_gridmap_costmap_generator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(filters REQUIRED)
find_package(grid_map_cmake_helpers REQUIRED)
find_package(grid_map_core REQUIRED)
find_package(grid_map_cv REQUIRED)
find_package(grid_map_msgs REQUIRED)
find_package(grid_map_octomap REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(OCTOMAP REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(OpenCV REQUIRED
  COMPONENTS
  opencv_highgui
  CONFIG
)

grid_map_package()

set(dependencies
  filters
  grid_map_msgs
  grid_map_ros
  rclcpp
  pluginlib
)

set(filter_libs
  obstacle_filter
)

include_directories(
  include
  SYSTEM
    ${EIGEN3_INCLUDE_DIR}
)
# ==================================
add_library(obstacle_filter SHARED src/ObstacleFilter.cpp)

foreach(lib_name ${filter_libs})
  ament_target_dependencies(${lib_name} SYSTEM
    ${dependencies}
  )

  if(CMAKE_CXX_COMPILER_ID MATCHES "GNU")
    target_compile_options(${lib_name} PRIVATE "SHELL:--param ggc-min-expand=1")
    target_compile_options(${lib_name} PRIVATE "SHELL:--param ggc-min-heapsize=32768")
  endif()
endforeach()

install(
  TARGETS ${filter_libs}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# ----------------------------------------------------------
add_executable(
  mapping_node
  src/mapping_node.cpp
)
add_library(mapping_lib SHARED
  src/Mapping.cpp
)
target_link_libraries(mapping_node mapping_lib)
ament_target_dependencies(
  mapping_lib SYSTEM
  ${dependencies}
  filters
)
# ----------------------------------------------------------
add_executable(
  image_to_gridmap_node
  src/image_to_gridmap_node.cpp
)
add_library(image_to_gridmap_lib SHARED
  src/ImageToGridmap.cpp
)
target_link_libraries(image_to_gridmap_node image_to_gridmap_lib)
ament_target_dependencies(
  image_to_gridmap_lib SYSTEM
  ${dependencies}
)
# ----------------------------------------------------------

set(targets_list
  mapping_node
  mapping_lib
  image_to_gridmap_node 
  image_to_gridmap_lib
)

foreach(target ${targets_list})
  if(CMAKE_CXX_COMPILER_ID MATCHES "GNU")
    target_compile_options(${target} PRIVATE "SHELL:--param ggc-min-expand=1")
    target_compile_options(${target} PRIVATE "SHELL:--param ggc-min-heapsize=32768")
  endif()
endforeach()

# Mark executables and/or libraries for installation
install(
  TARGETS ${targets_list}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install python scripts
install(
  PROGRAMS scripts/image_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)


# Mark cpp header files for installation
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.hpp"
)


# Mark other files for installation
install(
  DIRECTORY config data maps launch rviz
  DESTINATION share/${PROJECT_NAME}
)

install(
  FILES filter_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(${dependencies})
ament_export_libraries(${filter_libs})
pluginlib_export_plugin_description_file(filters filter_plugins.xml)
ament_package()
