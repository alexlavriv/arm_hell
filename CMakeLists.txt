# cmake_minimum_required(VERSION 2.8.3)
# project(arm_hell)


# ## Compile as C++11, supported in ROS Kinetic and newer
# # add_compile_options(-std=c++11)

# ## Find catkin macros and libraries
# ## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
# ## is used, also find other catkin packages

 

# add_executable(grabTheCan src/grabTheCan.cpp)
# target_link_libraries(grabTheCan ${catkin_LIBRARIES} ${Boost_LIBRARIES})

#  catkin_package()


cmake_minimum_required(VERSION 2.8.3)
project(arm_hell)
# Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS message_generation roscpp rospy genmsg std_msgs moveit_ros_planning_interface moveit_visual_tools cv_bridge  image_transport)
find_package( OpenCV REQUIRED )
 

 






include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(SYSTEM ${OpenCV_INCLUDE_DIRS})
include_directories(

  SYSTEM ${Boost_INCLUDE_DIR} 
  ${EIGEN_INCLUDE_DIRS}
)


## Declare ROS messages and services
# add_message_files(FILES Num.msg)
# add_service_files(FILES AddTwoInts.srv)

## Generate added messages and services

## Declare a catkin package
catkin_package()
 

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(grabTheCan src/grabTheCan.cpp)
target_link_libraries(grabTheCan ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${Boost_LIBRARIES})