cmake_minimum_required(VERSION 2.8)

SET(CMAKE_BUILD_TYPE "Release")
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")
add_compile_options(-std=c++11)
add_definitions(-std=c++11)

project(cam_fusion)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  pcl_ros
  geometry_msgs
  tf
  eigen_conversions
  message_generation 
  std_srvs
  cv_bridge
  sensor_msgs
  image_transport
)

catkin_package()

# include(cmake/PCL.cmake)
find_package(PCL REQUIRED QUIET)

set(OpenCV_DIR /usr/local/opencv-4.3.0/build)
find_package(OpenCV 4.3.0 REQUIRED)

include_directories(include 
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    )
link_directories(${OpenCV_LIBRARY_DIRS} ${PCL_LIBRARY_DIRS})
add_definitions(${OpenCV_DEFINITIONS})


# aux_source_directory(./src SRCS)
# add_executable(project_lidar_to_cam ./src/project_lidar_to_cam.cpp)
# target_link_libraries(project_lidar_to_cam ${OpenCV_LIBRARIES} ${PCL_LIBRARIES}) 

add_executable(remove_node ./src/apps/remove_node.cpp ./src/subscriber/cloud_subscriber.cpp ./src/publisher/cloud_publisher.cpp ./src/tools/image_converter.cpp ./src/remove_dynamic/remove_dynamic_flow.cpp)
target_link_libraries(remove_node ${OpenCV_LIBRARIES} ${PCL_LIBRARIES} ${catkin_LIBRARIES})