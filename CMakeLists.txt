cmake_minimum_required(VERSION 2.8.3)
project(hardware_driver)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake/") #find external libraries

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    cv_bridge
    image_transport
    roscpp
    roslaunch
    std_msgs
    message_generation
    )


string(REPLACE "/opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_core3.so.3.3.1;" "" catkin_LIBRARIES "${catkin_LIBRARIES}")
find_package(OpenCV REQUIRED
    NO_MODULE # should be optional, tells CMake to use config mode
    PATHS /usr/local # look here
    NO_DEFAULT_PATH) # and don't look anywhere else

find_package(Gphoto2 REQUIRED )
find_package(CURL REQUIRED)


# Generate services in the 'srv' folder
add_service_files(
    FILES
    DSLRCameraService.srv
    KapturaService.srv
    )

# Generate added messages and services with any dependencies listed here
generate_messages(
    DEPENDENCIES
    std_msgs
    )



catkin_package(
    INCLUDE_DIRS include
    #  LIBRARIES camera_driver
    #  CATKIN_DEPENDS cv_bridge image_transport roscpp roslaunch std_msgs
    #  DEPENDS system_lib
    )

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    SYSTEM ${OpenCV_LIBS}
    ${Gphoto2_INCLUDE_DIRS}
    ${CURL_INCLUDE_DIR}
    )
add_executable(dslr_node src/dslr_driver.cpp)
target_link_libraries(dslr_node ${catkin_LIBRARIES}
    ${Gphoto2_LIBRARIES}
    ${OpenCV_LIBS})
### Specify libraries to link a library or executable target against
add_executable(kaptura_service src/kaptura_service.cpp)
target_link_libraries(kaptura_service ${catkin_LIBRARIES}
    ${CURL_LIBRARIES})
############
add_executable(dslr_client src/dslr_client.cpp)
target_link_libraries(dslr_client ${catkin_LIBRARIES}
    ${Gphoto2_LIBRARIES}
    ${OpenCV_LIBS}
    ${CURL_LIBRARIES})
