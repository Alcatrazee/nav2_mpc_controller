cmake_minimum_required(VERSION 3.8)
project(nav2_mpc_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(rclcpp REQUIRED)
find_package(nav2_common REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(casadi REQUIRED)
find_package(visualization_msgs REQUIRED)

nav2_package()
set(CMAKE_CXX_STANDARD 17)

include_directories(
  include
)

set(library_name nav2_mpc_controller)



set(dependencies
  rclcpp
  nav2_common
  nav2_core
  nav2_util
  nav2_costmap_2d
  geometry_msgs
  nav_msgs
  nav2_msgs
  pluginlib
  tf2
  tf2_geometry_msgs
  tf2_ros
  tf2_eigen
  visualization_msgs
)

set(library_name ${library_name})

# Controller library
add_library(${library_name} SHARED
        src/mpc_controller.cpp
        src/transform_tools.cpp
        src/ref_trajectory_generator.cpp
        src/trajectory_profiler.cpp
        src/local_planner.cpp
        src/viz_function.cpp)

ament_target_dependencies(${library_name}
  ${dependencies}
)
target_link_libraries(${library_name} casadi)

install(TARGETS ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

# install plugin description
install(FILES
  mpc_controller.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})

pluginlib_export_plugin_description_file(nav2_core mpc_controller.xml)

ament_package()
