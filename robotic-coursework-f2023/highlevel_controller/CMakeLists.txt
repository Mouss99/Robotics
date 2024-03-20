cmake_minimum_required(VERSION 3.0.2)
project(highlevel_controller)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_definitions(-Wall -Werror)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  highlevel_msgs
  kinematic_controller
  cubic_polynomial_planner
  actionlib
)

find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS    
  ${Eigen3_INCLUDE_DIRS}
  LIBRARIES 
  CATKIN_DEPENDS
    roscpp
    highlevel_msgs
    kinematic_controller
    cubic_polynomial_planner
    actionlib 
  DEPENDS
 
)

include_directories(
	include
	${EIGEN3_INCLUDE_DIRS}	
	${catkin_INCLUDE_DIRS}	
)

add_executable(${PROJECT_NAME}
	src/client.cpp
  	src/action_client.cpp  
)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} )



#############
## Install ##
#############

# Mark cpp header files for installation
# Mark cpp header files for installation
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
)
