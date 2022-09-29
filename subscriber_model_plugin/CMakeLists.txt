cmake_minimum_required(VERSION 3.0.2)
project(subscriber_model_plugin)

find_package(gazebo REQUIRED)    #for gazebo plugins

find_package(catkin REQUIRED COMPONENTS
  gazebo_plugins                    #for gazebo plugins
  gazebo_ros                        #for gazebo ros plugins
  roscpp
)

catkin_package()

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

#for gazebo plugins
include_directories(${GAZEBO_INCLUDE_DIRS})            
link_directories(${GAZEBO_LIBRARY_DIRS})               
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

# for compiling plugin
add_library(gazebo_subscriber SHARED src/gazebo_subscriber.cc)
target_link_libraries(gazebo_subscriber ${GAZEBO_LIBRARIES})




