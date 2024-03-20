cmake_minimum_required(VERSION 3.0.2)
project(kinematic_controller)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


## Find catkin macros and libraries
find_package(catkin 
	REQUIRED
	COMPONENTS
		roscpp
		geometry_msgs
		highlevel_msgs
)

find_package(Eigen3 REQUIRED)						# find eigen3
find_package(pinocchio REQUIRED)					# find pinocchio
add_definitions("-DBOOST_MPL_LIMIT_LIST_SIZE=30") 	# something for pinocchio



catkin_package(
	INCLUDE_DIRS
	#${catkin_INCLUDE_DIRS}
    #	include  
    	${Eigen3_INCLUDE_DIRS}
    	${PINOCCHIO_INCLUDE_DIRS}
  	LIBRARIES
  		pinocchio    	  		
  	CATKIN_DEPENDS
    	roscpp
    	geometry_msgs
		highlevel_msgs    	 	    
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
	${PINOCCHIO_INCLUDE_DIRS}
)

add_executable(joint_controller
		src/joint.cpp
		src/joint_controller.cpp
)
target_link_libraries(joint_controller
  	${catkin_LIBRARIES}
	${PINOCCHIO_LIBRARIES}
)

add_executable(inverse_kinematic_controller
		src/task.cpp
		src/task_controller.cpp
)
target_link_libraries(inverse_kinematic_controller
  	${catkin_LIBRARIES}
	${PINOCCHIO_LIBRARIES}
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
