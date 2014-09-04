cmake_minimum_required(VERSION 2.4.6)
if(COMMAND cmake_policy)
    cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)

project(baxter_demos)

set(GCC_COVERAGE_COMPILE_FLAGS "-g" )
set(CMAKE_C_FLAGS ${GCC_COVERAGE_COMPILE_FLAGS} )

find_package(OpenCV REQUIRED)

find_package(Boost)

find_package(catkin REQUIRED COMPONENTS
    nodelet
    roscpp
    rospy
    baxter_interface
    std_msgs
    geometry_msgs
    moveit_msgs
    message_generation
    pcl_conversions
    pcl_ros
    tf
)

catkin_python_setup()

add_message_files(
    FILES
    BlobInfo.msg
    BlobInfoArray.msg
    CollisionObjectArray.msg
)

generate_messages(
    DEPENDENCIES
    std_msgs
    geometry_msgs
    moveit_msgs
)

catkin_package(
    CATKIN_DEPENDS message_runtime roscpp cv_bridge sensor_msgs moveit_msgs pcl_conversions tf
    DEPENDS OpenCV2
)

install(
  DIRECTORY scripts/
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  USE_SOURCE_PERMISSIONS
)

install(
  DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  USE_SOURCE_PERMISSIONS
)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(PCL 1.7.2 COMPONENTS common io filters segmentation search visualization features)
if(PCL_FOUND)
  include_directories(include)
  include_directories(include/impl)
  set(HEADER_FILES include/impl/CloudSegmenter.cpp include/CloudSegmenter.h include/OrientedBoundingBox.h)
  add_library(segmenter ${HEADER_FILES})
  target_link_libraries(segmenter ${PCL_LIBRARIES} ${catkin_LIBRARIES} ${boost_libraries})
  add_executable(ColorPicker src/ColorPicker.cpp)
  target_link_libraries(ColorPicker segmenter)
else()
  message("Couldn't find PCL version 1.7.2, so not compiling 3D segmentation support.")
endif()

