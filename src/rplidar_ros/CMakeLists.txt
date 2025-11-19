cmake_minimum_required(VERSION 3.0.2)
project(rplidar_ros)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rosconsole
  sensor_msgs
  std_srvs
)

catkin_package(
  CATKIN_DEPENDS roscpp rosconsole sensor_msgs std_srvs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# Check if SLAMTEC SDK is available
set(RPLIDAR_SDK_PATH "${CMAKE_CURRENT_SOURCE_DIR}/sdk")
if(EXISTS "${RPLIDAR_SDK_PATH}/include/sl_lidar.h")
  message(STATUS "SLAMTEC SDK found at ${RPLIDAR_SDK_PATH}")
  
  # Add SDK include directories
  include_directories(
    ${RPLIDAR_SDK_PATH}/include
    ${RPLIDAR_SDK_PATH}/src
  )
  
  # Collect SDK source files
  file(GLOB RPLIDAR_SDK_SRC
    "${RPLIDAR_SDK_PATH}/src/arch/linux/*.cpp"
    "${RPLIDAR_SDK_PATH}/src/dataunpacker/*.cpp"
    "${RPLIDAR_SDK_PATH}/src/dataunpacker/unpacker/*.cpp"
    "${RPLIDAR_SDK_PATH}/src/hal/*.cpp"
    "${RPLIDAR_SDK_PATH}/src/*.cpp"
  )
  
  # Add compile definition to enable SDK usage
  add_definitions(-DUSE_SLAMTEC_SDK)
  
  # Build with SDK
  add_executable(rplidarNode src/rplidar_node.cpp ${RPLIDAR_SDK_SRC})
else()
  message(WARNING "SLAMTEC SDK not found. Building without SDK support.")
  message(WARNING "The node will not function without the SDK.")
  message(WARNING "Please add the SDK to ${RPLIDAR_SDK_PATH}")
  
  # Build without SDK (will show warnings but compile)
  add_executable(rplidarNode src/rplidar_node.cpp)
endif()

target_link_libraries(rplidarNode ${catkin_LIBRARIES})

add_executable(rplidarNodeClient src/rplidar_client.cpp)
target_link_libraries(rplidarNodeClient ${catkin_LIBRARIES})

install(TARGETS rplidarNode rplidarNodeClient
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)

