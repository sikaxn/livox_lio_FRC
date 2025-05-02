cmake_minimum_required(VERSION 3.10)
project(lio_livox)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c11")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++20")
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

set(PACKAGES
	SuiteSparse
	Ceres
    Eigen3
	rclcpp
	message_filters
    geometry_msgs
    nav_msgs
	sensor_msgs
	std_msgs
	visualization_msgs
    std_srvs
    tf2_ros
	rosbag2
	livox_ros_driver2
    pcl_conversions
    pcl_ros
)

# FIND PACKAGES
foreach(package ${PACKAGES})
  find_package(${package} REQUIRED)
  if(!${package}_FOUND)
    message("not found package: ${package}")
  endif()
endforeach()

include_directories(
	include
	${EIGEN3_INCLUDE_DIR}
	${PCL_INCLUDE_DIRS}
	${CERES_INCLUDE_DIRS}
	${SUITESPARSE_INCLUDE_DIRS}
)


add_executable(
	ScanRegistration 
	src/lio/ScanRegistration.cpp
	src/lio/LidarFeatureExtractor.cpp
	src/segment/segment.cpp
	src/segment/pointsCorrect.cpp
)

target_link_libraries(
	ScanRegistration
	${CERES_LIBRARIES}
)

ament_target_dependencies(
	ScanRegistration
	${PACKAGES}
)

add_executable(
	PoseEstimation 
	src/lio/PoseEstimation.cpp
	src/lio/Estimator.cpp
	src/lio/IMUIntegrator.cpp
	src/lio/ceresfunc.cpp 
	src/lio/Map_Manager.cpp
)

target_link_libraries(
	PoseEstimation
	${CERES_LIBRARIES}
)

ament_target_dependencies(
	PoseEstimation
	${PACKAGES}
)

install(
  TARGETS ScanRegistration PoseEstimation
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()