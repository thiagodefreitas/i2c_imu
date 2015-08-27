cmake_minimum_required(VERSION 2.8.3)
project(i2c_imu)

find_package(catkin REQUIRED COMPONENTS
  sensor_msgs
  roscpp
  tf
)

catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES rtimu
  CATKIN_DEPENDS sensor_msgs roscpp tf
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(i2c_imu_node src/i2c_imu_node.cpp)

target_link_libraries(i2c_imu_node
  RTIMULib
  ${catkin_LIBRARIES}
)

 install(TARGETS i2c_imu_node
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

install(DIRECTORY 
   launch param
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
 )
