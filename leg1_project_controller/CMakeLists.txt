cmake_minimum_required(VERSION 3.0.2)
project(leg1_project_controller)

find_package(catkin REQUIRED 
  COMPONENTS
  roscpp
)

catkin_package(
  INCLUDE_DIRS include
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION} )

