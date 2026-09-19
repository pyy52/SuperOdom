cmake_minimum_required(VERSION 3.5)
project(super_odometry_vio)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
add_compile_options(-Wall -Wextra)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

# ---------------------------------------------------------------------------
# camodocal camera-model library, imported from VINS-Mono camera_model package
# (upstream HKUST-Aerial-Robotics/VINS-Mono @ 90dabb5e, GPLv3).
# Offline-calibration sources (chessboard/, calib/) are not used by odometry
# and are intentionally excluded.
# ---------------------------------------------------------------------------
set(CAMODOCAL_SOURCES
  src/camera_models/Camera.cc
  src/camera_models/CameraFactory.cc
  src/camera_models/CataCamera.cc
  src/camera_models/CostFunctionFactory.cc
  src/camera_models/EquidistantCamera.cc
  src/camera_models/PinholeCamera.cc
  src/camera_models/ScaramuzzaCamera.cc
  src/gpl/EigenQuaternionParameterization.cc
  src/gpl/gpl.cc
)

add_library(camodocal SHARED ${CAMODOCAL_SOURCES})
target_include_directories(camodocal PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(camodocal PUBLIC
  ${OpenCV_LIBS}
  ${CERES_LIBRARIES}
  Boost::boost
)

ament_export_targets(camodocal HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(Eigen3 OpenCV Ceres Boost)

# ---------------------------------------------------------------------------
# VINS-Mono feature-tracking core (upstream 90dabb5e, GPLv3). ROS1 parameter
# access and logging are shimmed out; see src/tracker/tracker_log.h and
# docs/THIRD_PARTY.md.
# ---------------------------------------------------------------------------
add_library(vins_tracker STATIC
  src/tracker/feature_tracker.cpp
  src/tracker/parameters.cpp
)
target_include_directories(vins_tracker PUBLIC src/tracker)
target_link_libraries(vins_tracker PUBLIC camodocal ${OpenCV_LIBS})

# ROS2 adapter: image in -> VINS-format feature PointCloud out.
add_executable(vio_feature_tracker_node src/ros2/tracker_node.cpp)
ament_target_dependencies(vio_feature_tracker_node
  rclcpp sensor_msgs geometry_msgs cv_bridge)
target_link_libraries(vio_feature_tracker_node vins_tracker)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS vio_feature_tracker_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME}/)
install(TARGETS camodocal EXPORT camodocal
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest QUIET)
  if(ament_cmake_gtest_FOUND)
    ament_add_gtest(test_camera_model test/test_camera_model.cpp)
    target_link_libraries(test_camera_model camodocal Eigen3::Eigen)

    ament_add_gtest(test_feature_tracker test/test_feature_tracker.cpp)
    target_link_libraries(test_feature_tracker vins_tracker camodocal Eigen3::Eigen)
  endif()
endif()

ament_package()
