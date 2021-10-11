cmake_minimum_required(VERSION 3.16)
project(franka_pivot_control_ros)

SET(CMAKE_CXX_STANDARD 17)
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/CMake")

find_package(catkin REQUIRED COMPONENTS
        roscpp
        pivot_control_messages_ros
        geometry_msgs
        tf2
        tf2_ros
        tf2_geometry_msgs
        )

if (TARGET franka_pivot_control)
    message("found franka_pivot_control")
else()
    add_subdirectory(franka_pivot_control)
endif()

catkin_package(
#        INCLUDE_DIRS include
        #  LIBRARIES esvr2_ros
        CATKIN_DEPENDS roscpp
        #  DEPENDS system_lib
)

set( CMAKE_CURRENT_BIN_DIR "${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}" )
message(franka_pivot_control_INCLUDE_DIR:${franka_pivot_control_INCLUDE_DIR})
include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

set( CMAKE_CURRENT_BIN_DIR "${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}" )

macro( add_recursive dir retVal )
    file( GLOB_RECURSE ${retVal} ${dir}/*.h ${dir}/*.cpp ${dir}/*.c )
endmacro()

add_recursive(src SOURCES)

add_executable( ${PROJECT_NAME} ${SOURCES} )

add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

target_link_libraries( ${PROJECT_NAME}
        ${catkin_LIBRARIES}
        franka_pivot_control
        pivot_control_messages
        pivot_control_messages_ros)
