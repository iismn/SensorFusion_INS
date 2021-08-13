cmake_minimum_required(VERSION 2.8.3)
project(sensor_fusion_smoothing)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -pthread")

find_package(catkin REQUIRED COMPONENTS
  # ROS MAIN
  tf
  roscpp
  rospy
  cv_bridge

  # ROS MES
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  message_generation
  visualization_msgs

  # PCL LIBRARIES
  pcl_conversions

  # CUDA - P2P REG
  # ndt_gpu
  fast_gicp
)

find_package(OpenMP REQUIRED)
find_package(PCL REQUIRED QUIET)
find_package(OpenCV REQUIRED QUIET)
find_package(GTSAM REQUIRED QUIET)
# CUDA #############################################
find_package(CUDA)
include_directories(${CUDA_INCLUDE_DIRS})
link_directories(${CUDA_LIBRARY_DIRS})
add_definitions(-DCUDA_FOUND)
add_definitions(-DUSE_VGICP_CUDA)


add_message_files(
  DIRECTORY msg
  FILES
  cloud_info.msg
)

add_service_files(
  DIRECTORY srv
  FILES
  save_map.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
  nav_msgs
  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS include
  DEPENDS PCL GTSAM

  CATKIN_DEPENDS
  std_msgs
  nav_msgs
  geometry_msgs
  sensor_msgs
  message_runtime
  message_generation
  visualization_msgs
)

# include directories
include_directories(
	include
	${catkin_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
	${GTSAM_INCLUDE_DIR}
)

# link directories
link_directories(
	include
	${PCL_LIBRARY_DIRS}
  ${OpenCV_LIBRARY_DIRS}
  ${GTSAM_LIBRARY_DIRS}
)

###########
## Build ##
###########

# Mapping Optimization
add_executable(${PROJECT_NAME}_LocalMapOptimization src/LocalMAP_OSM_Localization.cpp)
add_dependencies(${PROJECT_NAME}_LocalMapOptimization ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_compile_options(${PROJECT_NAME}_LocalMapOptimization PRIVATE ${OpenMP_CXX_FLAGS})
target_include_directories(${PROJECT_NAME}_LocalMapOptimization PUBLIC ${CUDA_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME}_LocalMapOptimization ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${OpenMP_CXX_FLAGS} gtsam fast_gicp)
