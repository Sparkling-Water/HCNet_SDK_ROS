cmake_minimum_required(VERSION 3.5.0)
project(hcnet_sdk)

set(CMAKE_BUILD_TYPE "Release")
#set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_FLAGS_DEBUG "-O3 -Wall -g -pthread")

find_package(catkin REQUIRED COMPONENTS
    roslaunch
    tf
    roscpp
    cv_bridge
    image_transport
    # msgs
    std_msgs
    sensor_msgs
    geometry_msgs
    nav_msgs
    message_generation
    visualization_msgs
)
find_package(OpenCV REQUIRED QUIET)

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS
    std_msgs
    nav_msgs
    geometry_msgs
    sensor_msgs
    visualization_msgs
    message_runtime
)

# include directories
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

# link directories
link_directories(
    include
    lib
    ${OpenCV_LIBRARY_DIRS}
)

# 设置依赖库
set(depends)
# hikvision
include_directories(/home/ssj/software/HCNetSDK/include)
foreach(module AudioRender HCCore hcnetsdk hpr PlayCtrl)
find_library(HIK_${module}
    NAMES ${module}
    HINTS /home/ssj/software/HCNetSDK/lib
)
list(APPEND depends ${HIK_${module}})
endforeach()

###########
## Build ##
###########

file(GLOB_RECURSE SOURCE_FILES
    src/*.cpp
)

add_executable(${PROJECT_NAME}  ${SOURCE_FILES})
target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${depends}
)