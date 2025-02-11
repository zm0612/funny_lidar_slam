cmake_minimum_required(VERSION 3.16)
set(PCL_ROOT "/usr")
project(funny_lidar_slam)
#set(CMAKE_VERBOSE_MAKEFILE true)

IF (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
ENDIF ()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -DDEBUG")

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_definitions(-DPROJECT_SOURCE_DIR=\"${PROJECT_SOURCE_DIR}\")

# 编译选项
add_compile_options(-Wall -O3)

# Eigen
include(cmake/FindEigen.cmake)
include_directories(${EIGEN3_INCLUDE_DIRS})

# Glog
include(cmake/glog.cmake)
include_directories(${GLOG_INCLUDE_DIRS})

# g2o
include(cmake/g2o.cmake)
include_directories(${G2O_INCLUDE_DIRS})


find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
include_directories(${CMAKE_SOURCE_DIR})

# livox message
find_package(livox_ros_driver2 QUIET)
if(NOT livox_ros_driver2_FOUND)
  message(FATAL_ERROR "Not Found Livox ROS 2 Driver")
endif()

################################################################################
# Declare ROS messages, services and actions
################################################################################

file(GLOB_RECURSE srv_files  RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} srv/*.srv)
# generate custom msg headers
set(MSG_INTERFACES ${PROJECT_NAME}_interfaces)
rosidl_generate_interfaces(${MSG_INTERFACES}
  ${srv_files}
  DEPENDENCIES builtin_interfaces std_msgs
  LIBRARY_NAME ${PROJECT_NAME}
)

include_directories(${CMAKE_SOURCE_DIR}/include
)

ament_auto_add_library(
    funny_lidar_slam_lib SHARED
    include/common/keyframe.cpp
    src/optimization/g2o/position_edge.cpp
    src/optimization/g2o/velocity_edge.cpp
    src/optimization/g2o/rotation_edge.cpp
    src/optimization/g2o/gyro_bias_rw_edge.cpp
    src/optimization/g2o/relative_pose_edge.cpp
    src/optimization/g2o/accel_bias_rw_edge.cpp
    src/optimization/g2o/gyro_bias_prior_edge.cpp
    src/optimization/g2o/accel_bias_prior_edge.cpp
    src/optimization/g2o/prior_nav_state_edge.cpp
    src/optimization/g2o/pre_integration_edge.cpp
    src/optimization/g2o/loopclosure_optimizer.cpp
    src/ivox_map/ivox_map.cpp
    src/ivox_map/voxel_grid_node.cpp
    src/imu/pre_integration.cpp
    src/imu/data_synchronizer.cpp
    src/lidar/lidar_model.cpp
    src/lidar/lidar_distortion_corrector.cpp
    src/loam/feature_extractor.cpp
    src/loam/pointcloud_projector.cpp
    src/slam/config_parameters.cpp
    src/slam/preprocessing.cpp
    src/slam/localization.cpp
    src/slam/loop_closure.cpp
    src/slam/split_map.cpp
    src/slam/frontend.cpp
    src/slam/system.cpp
)

# include file direcotry
target_include_directories(funny_lidar_slam_lib PUBLIC
   ${MSG_INTERFACES_INCLUDE_DIRECTORIES}   # for custom msgs
)

target_link_libraries(
    funny_lidar_slam_lib
    ${MSG_INTERFACE_TARGET}   # for custom msgs
    ${GLOG_LIBRARIES}
    ${YAML_CPP_LIBRARIES}
    ${FLANN_LIBRARY}
    ${G2O_LIBRARIES}
)

rosidl_get_typesupport_target(cpp_typesupport_target
   ${MSG_INTERFACES} "rosidl_typesupport_cpp")
target_link_libraries(funny_lidar_slam_lib "${cpp_typesupport_target}")

ament_auto_add_executable(funny_lidar_slam app/lidar_slam_app.cpp)
target_link_libraries(funny_lidar_slam funny_lidar_slam_lib)


### TEST ###
option(build_unit_tests "will execute all unit tests" ON)

if (build_unit_tests)
    enable_testing()
    add_subdirectory(test)
endif ()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(INSTALL_TO_SHARE
  config
  launch
)
