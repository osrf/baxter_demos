cmake_minimum_required(VERSION 2.4.6)
if(COMMAND cmake_policy)
    cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)

project(baxter_demos)

set(GCC_COVERAGE_COMPILE_FLAGS "-g" )

find_package(OpenCV REQUIRED)

find_package(PCL 1.7.2 REQUIRED COMPONENTS common io filters segmentation search visualization features)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    baxter_interface
    std_msgs
    geometry_msgs
    moveit_msgs
    message_generation
    pcl_conversions
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

include_directories(include)
include_directories(include/impl)
set(HEADER_FILES include/impl/CloudSegmenter.cpp include/CloudSegmenter.h include/OrientedBoundingBox.h)
add_library(segmenter ${HEADER_FILES})

# don't actually need OpenCV yet
add_executable(object_finder_3d src/object_finder_3d.cpp)
#add_executable(MOI_test src/MOI_test.cpp)
target_link_libraries(object_finder_3d segmenter)
target_link_libraries(object_finder_3d ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES} ${PCL_FEATURE_LIBRARIES})
#target_link_libraries(MOI_test ${catkin_LIBRARIES} ${PCL_LIBRARIES})
