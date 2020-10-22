cmake_minimum_required(VERSION 3.0.2)
project(sim_vehicle_plugins)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS gazebo_ros roscpp geometry_msgs std_msgs tf)
find_package(Eigen3 REQUIRED)
find_package(gazebo REQUIRED)

include_directories(include ${catkin_INCLUDE_DIRS})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

catkin_package()

add_library(model2gazebo SHARED src/model2gazebo.cpp
								src/drone_model.cpp)

target_link_libraries(model2gazebo ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES} )

install(DIRECTORY include/
  	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  	FILES_MATCHING PATTERN "*.hh"
)

install(TARGETS
	model2gazebo
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)