cmake_minimum_required(VERSION 3.5)
project(usb_cam)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
 rospy
 cv_bridge message_generation image_transport roscpp std_msgs std_srvs sensor_msgs camera_info_manager)

# Find OpenCV
find_package(OpenCV REQUIRED) 

## pkg-config libraries
find_package(PkgConfig REQUIRED)
pkg_check_modules(avcodec libavcodec REQUIRED)
pkg_check_modules(avutil libavutil REQUIRED)
pkg_check_modules(swscale libswscale REQUIRED)
pkg_check_modules(udev libudev REQUIRED)

add_service_files(
  FILES
  SetExposureAuto.srv 
  SetExposureAbsolute.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)


###################################################
## Declare things to be passed to other projects ##
###################################################
## Generate services in the 'srv' folder

## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)


###########
## Build ##
###########

## Build the USB camera library
add_library(${PROJECT_NAME} src/usb_cam.cpp src/see3cam_hidraw.cpp)
target_link_libraries(${PROJECT_NAME} PUBLIC
  ${avcodec_LIBRARIES}
  ${avutil_LIBRARIES}
  ${swscale_LIBRARIES}
  ${catkin_LIBRARIES}
  ${udev_LIBRARIES}
  ${OpenCV_LIBS}
)

target_include_directories(${PROJECT_NAME} PUBLIC 
  ${catkin_INCLUDE_DIRS}
  ${avcodec_INCLUDE_DIRS}
  ${swscale_INCLUDE_DIRS}
  ${avutil_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  include
)

## Declare a cpp executable
add_executable(${PROJECT_NAME}_node nodes/usb_cam_node.cpp)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME})

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}_node ${PROJECT_NAME}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

## Copy launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)

install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)
