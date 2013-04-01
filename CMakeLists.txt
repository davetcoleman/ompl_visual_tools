cmake_minimum_required(VERSION 2.8.3)
project(ompl_rviz_viewer)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS 
  roscpp 
 roslib 
  visualization_msgs 
  ompl
)

find_package(ompl)

# Add cpp files 
set( sources
  src/ppm.cpp
  src/ompl_rviz_viewer.cpp 
)


#find_package(Boost REQUIRED COMPONENTS thread)
include_directories(${Boost_INCLUDE_DIRS} ${OMPL_INCLUDE_DIRS})

## LIBRARIES: libraries you create in this project that dependent 
## projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  DEPENDS 
  CATKIN_DEPENDS 
  INCLUDE_DIRS 
  LIBRARIES 
)

# Build
add_executable(ompl_rviz_viewer ${sources} )
target_link_libraries(ompl_rviz_viewer ${OMPL_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})
