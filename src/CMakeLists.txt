cmake_minimum_required(VERSION 2.8.3)
project(calibration_test)

set(CMAKE_CXX_STANDARD 14)


set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -g -O3")

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
  pcl_conversions
  pcl_ros

)

find_package(OpenCV REQUIRED)
find_package (Eigen3 3.3 REQUIRED NO_MODULE)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(calibration_test
        src/main.cpp
        src/CalibrationTest.cpp include/PointCloudTypes.h)

add_dependencies(calibration_test
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(calibration_test
  Eigen3::Eigen
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

