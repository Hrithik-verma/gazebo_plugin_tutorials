cmake_minimum_required(VERSION 3.0.2)
project(model_plugin_ros)

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
add_library(simple_model_ros_plugin SHARED src/simple_model_ros_plugin.cc)
target_link_libraries(simple_model_ros_plugin ${GAZEBO_LIBRARIES})


add_library(model_ros_plugin SHARED src/model_ros_plugin.cc)
target_link_libraries(model_ros_plugin ${GAZEBO_LIBRARIES})




