cmake_minimum_required(VERSION 2.8.3)
project(franka_pivot_control_ros)

add_compile_options(-std=c++14)
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/CMake")

find_package(catkin REQUIRED COMPONENTS
        roscpp
        pivot_control_messages_ros
        )

SET(franka_pivot_control_HOME)
SET(franka_pivot_control_LIBRARIES ${franka_pivot_control_HOME}/lib/libfranka_pivot_control.so)
SET(franka_pivot_control_INCLUDE_DIR ${franka_pivot_control_HOME}/include)

if (TARGET pivot_control_messages)
    message("found pivot_control_messages")
else()
    add_subdirectory(pivot_control_messages)
endif()

catkin_package(
        INCLUDE_DIRS include
        #  LIBRARIES esvr2_ros
        CATKIN_DEPENDS roscpp
        #  DEPENDS system_lib
)

set( CMAKE_CURRENT_BIN_DIR "${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}" )

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${franka_pivot_control_INCLUDE_DIR}
        pivot_control_messages/include
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
        ${franka_pivot_control_LIBRARIES}
        pivot_control_messages
        pivot_control_messages_ros)
