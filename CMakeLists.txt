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
        franka_control
        franka_msgs
        )

catkin_package(
#                INCLUDE_DIRS include
        #  LIBRARIES esvr2_ros
        LIBRARIES ${PROJECT_NAME}
)

if (TARGET franka_pivot_control)
    message("found franka_pivot_control")
else()
    add_subdirectory(franka_pivot_control)
endif()


set( CMAKE_CURRENT_BIN_DIR "${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}" )
include_directories(
        "include"
        "franka_pivot_control/include"
        "${catkin_INCLUDE_DIRS}"
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
        )
