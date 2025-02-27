cmake_minimum_required(VERSION 3.10)
project(orb_slam3_ros_wrapper)

# Set C++ standard (and any flags as needed)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE "Release")
add_compile_options(-Wall -O3 -g)

# Define ORB_SLAM3 directory (adjust if needed)
set(ORB_SLAM3_DIR $ENV{HOME}/Documents/ORB_SLAM3)

# Find ament and ROS2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED) 
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(sensor_msgs REQUIRED)

# Find 3rd-party libraries (OpenCV, Eigen3, Pangolin)
find_package(OpenCV REQUIRED)
message(STATUS "OPENCV VERSION: ${OpenCV_VERSION}")
find_package(Eigen3 REQUIRED)
message(STATUS "Eigen3 VERSION: ${Eigen3_VERSION}")
find_package(Pangolin REQUIRED)

include_directories(
  ${ORB_SLAM3_DIR}
  ${ORB_SLAM3_DIR}/include
  ${ORB_SLAM3_DIR}/include/CameraModels
  ${ORB_SLAM3_DIR}/Thirdparty/Sophus
  ${PROJECT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIRS}
  ${Pangolin_INCLUDE_DIRS}
)

# Define libraries to link
set(LIBS
  ${ORB_SLAM3_DIR}/lib/libORB_SLAM3.so
  ${EIGEN3_LIBRARIES}  # Verify the variable name for Eigen3 libraries
  ${Pangolin_LIBRARIES}
  ${OpenCV_LIBS}
)

# Define executables and link libraries
add_executable(${PROJECT_NAME}_mono
  src/mono_node.cc
  src/common.cc
)
target_link_libraries(${PROJECT_NAME}_mono ${LIBS})
ament_target_dependencies(${PROJECT_NAME}_mono
  rclcpp
  std_msgs
  cv_bridge
  image_transport
  tf2_ros
  sensor_msgs
)

add_executable(${PROJECT_NAME}_stereo
  src/stereo_node.cc
  src/common.cc
)
target_link_libraries(${PROJECT_NAME}_stereo ${LIBS})
ament_target_dependencies(${PROJECT_NAME}_stereo
  rclcpp
  std_msgs
  cv_bridge
  image_transport
  tf2_ros
  sensor_msgs
)

add_executable(${PROJECT_NAME}_mono_inertial
  src/mono_inertial_node.cc
  src/common.cc
)
target_link_libraries(${PROJECT_NAME}_mono_inertial ${LIBS})
ament_target_dependencies(${PROJECT_NAME}_mono_inertial
  rclcpp
  std_msgs
  cv_bridge
  image_transport
  tf2_ros
  sensor_msgs
)

add_executable(${PROJECT_NAME}_stereo_inertial
  src/stereo_inertial_node.cc
  src/common.cc
)
target_link_libraries(${PROJECT_NAME}_stereo_inertial ${LIBS})
ament_target_dependencies(${PROJECT_NAME}_stereo_inertial
  rclcpp
  std_msgs
  cv_bridge
  image_transport
  tf2_ros
  sensor_msgs
)

add_executable(${PROJECT_NAME}_rgbd
  src/rgbd_node.cc
  src/common.cc
)
target_link_libraries(${PROJECT_NAME}_rgbd ${LIBS})
ament_target_dependencies(${PROJECT_NAME}_rgbd
  rclcpp
  std_msgs
  cv_bridge
  image_transport
  tf2_ros
  sensor_msgs
)

# Install targets and directories
install(TARGETS
  ${PROJECT_NAME}_mono
  ${PROJECT_NAME}_stereo
  ${PROJECT_NAME}_mono_inertial
  ${PROJECT_NAME}_stereo_inertial
  ${PROJECT_NAME}_rgbd
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/trajectory_server.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME trajectory_server
)

install(PROGRAMS
  scripts/image_playback_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME image_playback_node
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

set_target_properties(${PROJECT_NAME}_mono_inertial PROPERTIES INSTALL_RPATH "${ORB_SLAM3_DIR}/lib")
set_target_properties(${PROJECT_NAME}_mono PROPERTIES INSTALL_RPATH "${ORB_SLAM3_DIR}/lib")
set_target_properties(${PROJECT_NAME}_stereo PROPERTIES INSTALL_RPATH "${ORB_SLAM3_DIR}/lib")
set_target_properties(${PROJECT_NAME}_stereo_inertial PROPERTIES INSTALL_RPATH "${ORB_SLAM3_DIR}/lib")
set_target_properties(${PROJECT_NAME}_rgbd PROPERTIES INSTALL_RPATH "${ORB_SLAM3_DIR}/lib")

ament_package()
