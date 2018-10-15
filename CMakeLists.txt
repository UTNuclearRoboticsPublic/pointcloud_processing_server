cmake_minimum_required(VERSION 2.8.3)
project(pointcloud_processing_server)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  pcl_conversions 
  pcl_ros 
  roscpp 
  std_msgs 
  geometry_msgs
  sensor_msgs 
  message_generation
  tf
)

find_package(Boost REQUIRED COMPONENTS system)

# Generate messages in the 'msg' folder
add_message_files(
  FILES 
  pointcloud_task.msg 
  pointcloud_task_result.msg 
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  pointcloud_process.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES std_msgs geometry_msgs sensor_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES pointcloud_task_creation pointcloud_process_publisher pointcloud_utilities #pointcloud_subtraction
  CATKIN_DEPENDS geometry_msgs 
)

include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${pcl_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(pointcloud_processing_server src/server.cpp)
add_dependencies(
   pointcloud_processing_server ${pointcloud_processing_server_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(
  pointcloud_processing_server ${catkin_LIBRARIES}
)

add_executable(map_maker_pcs src/map_maker_pcs.cpp)
target_link_libraries( 
  map_maker_pcs ${catkin_LIBRARIES}
)
add_dependencies(
  map_maker_pcs ${pointcloud_processing_server_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}
)

add_executable(tester src/test.cpp)
target_link_libraries( 
  tester ${catkin_LIBRARIES}
)
add_dependencies(
  tester ${pointcloud_processing_server_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}
)

add_library(pointcloud_task_creation src/pointcloud_task_creation.cpp)
add_dependencies(pointcloud_task_creation ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_library(pointcloud_utilities src/pointcloud_utilities.cpp src/pointcloud_transforms.cpp)
add_dependencies(pointcloud_utilities ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

#add_library(pointcloud_subtraction src/pointcloud_subtraction.cpp)
#add_dependencies(pointcloud_subtraction ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_library(pointcloud_process_publisher src/pointcloud_process_publisher.cpp)
add_dependencies(pointcloud_process_publisher ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Install project namespaced headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE)