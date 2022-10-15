cmake_minimum_required(VERSION 3.0.2)
project(light_move_plugin)

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
add_library(light_move_model_plugin SHARED src/light_move_model_plugin.cc)
target_link_libraries(light_move_model_plugin ${GAZEBO_LIBRARIES})

