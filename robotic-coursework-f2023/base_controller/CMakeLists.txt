cmake_minimum_required(VERSION 3.10)
project(base_controller)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_definitions(-Wall -Werror)

## Find catkin macros and libraries
find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    geometry_msgs
    tf2_geometry_msgs 
)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(Boost REQUIRED)

catkin_package(
  INCLUDE_DIRS    
      ${catkin_INCLUDE_DIRS}
  LIBRARIES 
  CATKIN_DEPENDS
    roscpp
    geometry_msgs
    tf2_geometry_msgs
  DEPENDS
 
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}  
  ${EIGEN3_INCLUDE_DIRS} 
)

add_executable(    ${PROJECT_NAME} src/base_controller.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} Eigen3::Eigen)
#add_dependencies(        ${PROJECT_NAME}_client hello_service_gencpp)



#############
## Install ##
#############

# Mark executables and/or libraries for installation
install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
)