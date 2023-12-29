cmake_minimum_required(VERSION 2.8.3)
project(moveit_simple_controller_manager)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(Boost REQUIRED thread)

find_package(catkin COMPONENTS
  moveit_core
  pluginlib
  actionlib
  roscpp
  control_msgs
  action_controller
  REQUIRED)

# Prioritize the specific include directory.
include_directories($ENV{HOME}/catkin_ws/src/drone-nav/moveit_simple_controller_manager/include/moveit_simple_controller_manager)


# Other include directories.
include_directories(
  $ENV{HOME}/catkin_ws/src/drone-nav/action_controller/include
  ${catkin_INCLUDE_DIRS}
  include
  SYSTEM ${Boost_INCLUDE_DIR} 
  "../action_controller/msg_gen/cpp/include"   #WIM Changed
)

link_directories(${catkin_LIBRARY_DIRS})


generate_messages(
  DEPENDENCIES
  action_controller
)

catkin_package(
  LIBRARIES
  INCLUDE_DIRS
  DEPENDS
    moveit_core
    )

include_directories(include)

add_library(moveit_simple_controller_manager src/moveit_simple_controller_manager.cpp)
target_link_libraries(moveit_simple_controller_manager ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS moveit_simple_controller_manager LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(FILES moveit_simple_controller_manager_plugin_description.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
       )
