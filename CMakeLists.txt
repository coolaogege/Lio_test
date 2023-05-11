cmake_minimum_required(VERSION 2.8.3)
project(lili_om_rot)

## Compile as C++14, supported in ROS Kinetic and newer
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -O3")

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  nav_msgs
  sensor_msgs
  geometry_msgs
  message_generation
  pcl_ros
  pcl_conversions
)

find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)

find_package(PCL REQUIRED QUIET)
find_package(OpenCV REQUIRED QUIET)
# SET(GTSAM_DIR /home/cc/workspace/3rdparty/gtsam323/lib/cmake/GTSAM)
# find_package(GTSAM REQUIRED)
set(GTSAM_INCLUDE_DIR /home/cc/workspace/3rdparty/gtsam403/include)
set(GTSAM_LIBRARY_DIRS /home/cc/workspace/3rdparty/gtsam403/lib)
# set(GTSAM_INCLUDE_DIR /home/cc/workspace/3rdparty/gtsam400/include)
# set(GTSAM_LIBRARY_DIRS /home/cc/workspace/3rdparty/gtsam400/lib)
message(STATUS "gtsam lib: ${GTSAM_LIBRARY_DIRS}")

message(STATUS "Using catkin version ${catkin_VERSION}")
message(STATUS "Using Ceres version ${CERES_VERSION}")

add_message_files(
  DIRECTORY msg
  FILES
  cloud_info.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
    # LIBRARIES ${PROJECT_NAME}
    INCLUDE_DIRS include
    DEPENDS PCL 
    CATKIN_DEPENDS roscpp message_runtime std_msgs nav_msgs geometry_msgs
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${CERES_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${GTSAM_INCLUDE_DIR}
)

link_directories(
    include
    ${GTSAM_LIBRARY_DIRS}
    ${PCL_LIBRARY_DIRS}
    ${CERES_LIBRARY_DIRS}
    ${OpenCV_LIBRARY_DIRS}
)

## Declare a C++ executable

add_executable(Preprocessing src/Preprocessing.cpp)
target_link_libraries(Preprocessing ${catkin_LIBRARIES})

add_executable(FeatureExtract src/FeatureExtract.cpp)
target_link_libraries(FeatureExtract ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(LidarOdometry src/LidarOdometry.cpp)
target_link_libraries(LidarOdometry ${catkin_LIBRARIES} ${CERES_LIBRARIES})

add_executable(BackendFusion src/BackendFusion.cpp src/MarginalizationFactor.cpp)
target_link_libraries(BackendFusion ${catkin_LIBRARIES} ${CERES_LIBRARIES} gtsam)

#For Debug
# set(CMAKE_BUILD_TYPE "Debug")
