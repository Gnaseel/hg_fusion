cmake_minimum_required(VERSION 3.0.2)
project(hg_fusion)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  pcl_ros
  pcl_conversions
  message_generation
  geometry_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)




## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

generate_messages(
  DEPENDENCIES
  std_msgs geometry_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES fusion_project
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
#  DEPENDS system_lib
)



## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
add_executable(Boxing src/Boxing.cpp)
add_dependencies(Boxing ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(Boxing ${catkin_LIBRARIES})

add_executable(Prefiltering src/Prefiltering.cpp)
add_dependencies(Prefiltering ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(Prefiltering ${catkin_LIBRARIES})


#add_executable(Model_gen src/Model_gen.cpp)
#add_dependencies(Model_gen ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(Model_gen ${catkin_LIBRARIES})