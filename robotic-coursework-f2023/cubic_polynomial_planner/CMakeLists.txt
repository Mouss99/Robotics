cmake_minimum_required(VERSION 3.0.2)
project(cubic_polynomial_planner)


set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_definitions(-Wall -Werror)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  highlevel_msgs
  actionlib
)

find_package(Eigen3 REQUIRED)	

catkin_package(
  INCLUDE_DIRS    
  	#${catkin_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS}
  LIBRARIES 
  CATKIN_DEPENDS
    geometry_msgs
    roscpp
    highlevel_msgs
    actionlib
  DEPENDS
 
)


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
	include
	${catkin_INCLUDE_DIRS}		
	${EIGEN3_INCLUDE_DIRS}		
)

add_executable(			${PROJECT_NAME}_server src/cubic.cpp)
target_link_libraries(	${PROJECT_NAME}_server ${catkin_LIBRARIES})



add_executable(action_server
                src/action.cpp
                src/action_server.cpp
)
target_link_libraries(action_server
  	${catkin_LIBRARIES}
)



#############
## Install ##
#############

# Mark cpp header files for installation
install(
  	DIRECTORY include/${PROJECT_NAME}/
 	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  	FILES_MATCHING PATTERN "*.hpp"
)

