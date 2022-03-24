cmake_minimum_required(VERSION 2.8) # Just copying the subdirectory
project(roomE)

# NOTE: Add cpp files here:
add_executable(${PROJECT_NAME} 
    test.cpp 
    # img_test.cpp
    ArduinoSerial.cpp 
    Lidar_MRPT.cpp
    RoomeMap.cpp
    RoomeNav.cpp
    DriveTrain.cpp
    )

############## YDLIDAR SDK START#####################################
#Include directories
INCLUDE_DIRECTORIES(
  ${CMAKE_SOURCE_DIR}
  ${CMAKE_SOURCE_DIR}/YDLidar-SDK
  ${CMAKE_SOURCE_DIR}/YDLidar-SDK/src
  ${CMAKE_CURRENT_BINARY_DIR}/YDLidar-SDK
)
#Add YDLIDAR SDK sub project
add_subdirectory(YDLidar-SDK)

#Link your project to ydlidar_sdk library.
target_link_libraries(${PROJECT_NAME} ydlidar_sdk)

############## YDLIDAR SDK END#####################################


############## MRPT START######################################### 
find_package(MRPT 1.5 COMPONENTS obs slam nav gui)
message(STATUS "MRPT_VERSION: ${MRPT_VERSION}")
message(STATUS "MRPT_LIBRARIES: ${MRPT_LIBRARIES}")
target_link_libraries(${PROJECT_NAME} ${MRPT_LIBRARIES})
############## MRPT END###########################################

#make sure we run C++ 17
target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_17)
