cmake_minimum_required(VERSION 2.8.3)
project(ompl_rviz_viewer)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS 
  roscpp 
  roslib
  visualization_msgs 
)

find_package(OMPL REQUIRED)
find_package(Boost REQUIRED COMPONENTS serialization)

include_directories(${Boost_INCLUDE_DIRS} ${OMPL_INCLUDE_DIRS})

catkin_package(
  CATKIN_DEPENDS 
    roscpp
    roslib
    visualization_msgs 
  INCLUDE_DIRS 
    include
  LIBRARIES 
    ${PROJECT_NAME}_ppm
)

include_directories(
  ${catkin_INCLUDE_DIRS} 
  ${OMPL_INCLUDE_DIRS}
  include
)

## Build

# PPM Library for loading images
add_library(${PROJECT_NAME}_ppm
  src/utilities/ppm.cpp
)
target_link_libraries(${PROJECT_NAME}_ppm
  ${catkin_LIBRARIES} ${Boost_LIBRARIES}
)

# Rviz Viewer Library
add_library(${PROJECT_NAME}
  src/ompl_rviz_viewer.cpp
)
target_link_libraries(${PROJECT_NAME} 
  ${PROJECT_NAME}_ppm ${catkin_LIBRARIES} ${Boost_LIBRARIES}  
)

# Executable for SimpleSetup (in our case T-RRT)
# add_executable(ompl_rviz_planner
#   src/ompl_rviz_planner.cpp 
# )
# target_link_libraries(ompl_rviz_planner 
#   ${OMPL_LIBRARIES} 
#   ${catkin_LIBRARIES} 
#   ${Boost_LIBRARIES}
#   ${PROJECT_NAME}_ppm
# )

# Executable for ExperienceSetup
add_executable(ompl_rviz_lightning
  src/ompl_rviz_lightning.cpp 
)
target_link_libraries(ompl_rviz_lightning 
  ${OMPL_LIBRARIES} 
  ${catkin_LIBRARIES} 
  ${Boost_LIBRARIES}
  ${PROJECT_NAME} 
  ${PROJECT_NAME}_ppm
)

# Executable for ExperienceDatabaseTest
add_executable(experience_database_test
  src/experience_database_test.cpp 
)
target_link_libraries(experience_database_test 
  ${OMPL_LIBRARIES} 
  ${catkin_LIBRARIES} 
  ${Boost_LIBRARIES}
  ${PROJECT_NAME} 
  ${PROJECT_NAME}_ppm
)

# Install
install(TARGETS ${PROJECT_NAME}_ppm
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

# Mark executables for installation
install(TARGETS ompl_rviz_lightning experience_database_test #ompl_rviz_planner
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})