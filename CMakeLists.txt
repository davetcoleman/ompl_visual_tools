cmake_minimum_required(VERSION 2.8.3)
project(ompl_rviz_viewer)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS 
  roscpp 
  roslib
  visualization_msgs 
)

find_package(OMPL REQUIRED)

include_directories(${Boost_INCLUDE_DIRS} ${OMPL_INCLUDE_DIRS})

catkin_package(
  CATKIN_DEPENDS 
  INCLUDE_DIRS 
    ${OMPL_INCLUDE_DIRS}
  LIBRARIES 
    ${OMPL_LIBRARIES}
)

include_directories(${catkin_INCLUDE_DIRS}
		    ${OMPL_INCLUDE_DIRS})

link_directories(${Boost_LIBRARY_DIRS})
link_directories(${OMPL_LIBRARY_DIRS})

# Build
add_executable(ompl_rviz_planner
  src/utilities/ppm.cpp
  src/ompl_rviz_planner.cpp 
)
target_link_libraries(ompl_rviz_planner ${OMPL_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})
