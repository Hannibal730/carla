cmake_minimum_required(VERSION 3.5)
project(nav2_mppi_controller)

add_definitions(-DXTENSOR_ENABLE_XSIMD)
add_definitions(-DXTENSOR_USE_XSIMD)
add_definitions(-DXTENSOR_USE_OPENMP)

set(XTENSOR_USE_TBB 0)
set(XTENSOR_USE_OPENMP 1)
set(XTENSOR_USE_XSIMD 1)

# set(XTENSOR_DEFAULT_LAYOUT column_major)  # row_major, column_major
# set(XTENSOR_DEFAULT_TRAVERSAL row_major)  # row_major, column_major

find_package(ament_cmake REQUIRED)
find_package(xtensor REQUIRED)
find_package(xsimd REQUIRED)
find_package(OpenMP REQUIRED)

include_directories(
  include
)

set(dependencies_pkgs
  rclcpp
  nav2_common
  pluginlib
  tf2
  geometry_msgs
  visualization_msgs
  nav_msgs
  nav2_core
  nav2_costmap_2d
  nav2_util
  tf2_geometry_msgs
  tf2_eigen
  tf2_ros
)

foreach(pkg IN LISTS dependencies_pkgs)
  find_package(${pkg} REQUIRED)
endforeach()

nav2_package()

include(CheckCXXCompilerFlag)

# -march=native: this machine's CPU (i7-13700HX, AVX2+FMA+AVX-VNNI) — do not use
# this binary on other hardware. Replaces the manual flag checks below.
add_compile_options(-O3 -finline-limit=10000000 -ffp-contract=fast -ffast-math -march=native)

add_library(mppi_controller SHARED
  src/controller.cpp
  src/optimizer.cpp
  src/critic_manager.cpp
  src/trajectory_visualizer.cpp
  src/path_handler.cpp
  src/parameters_handler.cpp
  src/noise_generator.cpp
)

add_library(mppi_critics SHARED
  src/critics/obstacles_critic.cpp
  src/critics/cost_critic.cpp
  src/critics/goal_critic.cpp
  src/critics/goal_angle_critic.cpp
  src/critics/path_align_critic.cpp
  src/critics/path_align_legacy_critic.cpp
  src/critics/path_follow_critic.cpp
  src/critics/path_angle_critic.cpp
  src/critics/prefer_forward_critic.cpp
  src/critics/twirling_critic.cpp
  src/critics/constraint_critic.cpp
  src/critics/velocity_deadband_critic.cpp
)

set(libraries mppi_controller mppi_critics)

foreach(lib IN LISTS libraries)
  target_compile_options(${lib} PUBLIC -fconcepts)
  target_include_directories(${lib} PUBLIC ${xsimd_INCLUDE_DIRS} ${OpenMP_CXX_INCLUDE_DIRS})
  target_link_libraries(${lib} xtensor xtensor::optimize xtensor::use_xsimd OpenMP::OpenMP_CXX)
  ament_target_dependencies(${lib} ${dependencies_pkgs})
endforeach()

install(TARGETS mppi_controller mppi_critics
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
  add_subdirectory(test)
  # add_subdirectory(benchmark)
endif()

ament_export_libraries(${libraries})
ament_export_dependencies(${dependencies_pkgs})
ament_export_include_directories(include)
pluginlib_export_plugin_description_file(nav2_core mppic.xml)
pluginlib_export_plugin_description_file(nav2_mppi_controller critics.xml)

ament_package()
