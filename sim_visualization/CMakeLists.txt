cmake_minimum_required(VERSION 3.0.2)
project(sim_visualization)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp)

## Declare a catkin package
catkin_package( CATKIN_DEPENDS roscpp )

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(visualization src/main.cpp
    include/trace3D.h

    src/trace3D.cpp
)

target_link_libraries(visualization ${catkin_LIBRARIES} )
