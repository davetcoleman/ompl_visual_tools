cmake_minimum_required(VERSION 2.8.3)
project(ompl_visual_tools)

# C++ 11
set(CMAKE_CXX_FLAGS "-std=c++11 -Wall ${CMAKE_CXX_FLAGS}")

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  graph_msgs
  moveit_visual_tools
  moveit_core
  moveit_ompl
  roscpp
  roslib
  visualization_msgs
)

find_package(ompl REQUIRED)
find_package(Boost)

catkin_package(
  CATKIN_DEPENDS
    graph_msgs
    moveit_visual_tools
    moveit_core
    moveit_ompl
    roscpp
    roslib
    visualization_msgs
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
    ${PROJECT_NAME}_ppm
)

###########
## Build ##
###########

include_directories(SYSTEM
  ${Boost_INCLUDE_DIRS}
  ${OMPL_INCLUDE_DIRS}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OMPL_INCLUDE_DIRS}
)

# PPM Library for loading images
add_library(${PROJECT_NAME}_ppm
  src/utilities/ppm.cpp
)
target_link_libraries(${PROJECT_NAME}_ppm
  ${catkin_LIBRARIES} ${Boost_LIBRARIES}
)

# OMPL Visual Tools Library
add_library(${PROJECT_NAME}
  #src/ompl_visual_tools.cpp
  src/ros_viz_window.cpp
  src/moveit_viz_window.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${PROJECT_NAME}_ppm
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

# Executable for Demos and Testing
# add_executable(${PROJECT_NAME}_rrt_demo
#   src/demos/rrt_demo.cpp
# )
# set_target_properties(${PROJECT_NAME}_rrt_demo
#   PROPERTIES OUTPUT_NAME rrt_demo PREFIX ""
# )
# target_link_libraries(${PROJECT_NAME}_rrt_demo
#   ${OMPL_LIBRARIES}
#   ${catkin_LIBRARIES}
#   ${Boost_LIBRARIES}
#   ${PROJECT_NAME}
#   ${PROJECT_NAME}_ppm
# )

## Install

# Install libraries
install(TARGETS ${PROJECT_NAME}_ppm ${PROJECT_NAME}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

# Install header files
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

# Install shared resources
install(DIRECTORY launch    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY resources DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

# # Install executables
# install(TARGETS
#     ${PROJECT_NAME}_rrt_demo
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
