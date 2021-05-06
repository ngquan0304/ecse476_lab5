cmake_minimum_required(VERSION 2.8.3)
project(locomotion_action_server)

find_package(catkin_simple REQUIRED)
find_package(actionlib_msgs REQUIRED)

# @Matthew - Added for action lib
# add_action_files(
#   DIRECTORY src/action
#   FILES Locomotion.action
# )

catkin_simple()

# example boost usage
# find_package(Boost REQUIRED COMPONENTS system thread)

# C++0x support - not quite the same as final C++11!
# use carefully;  can interfere with point-cloud library
# SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

# Libraries: uncomment the following and edit arguments to create a new library
cs_add_library(traj_builder src/traj_builder.cpp)   

# Executables: uncomment the following and edit arguments to compile new nodes
# may add more of these lines for more nodes from the same package
cs_add_executable(lidar_alarm_mobot src/lidar_alarm_mobot.cpp)
cs_add_executable(current_state_publisher src/current_state_publisher.cpp)
#cs_add_executable(modal_trajectory_controller src/modal_trajectory_controller.cpp)
cs_add_executable(des_state_publisher_service src/des_state_publisher_service.cpp)
cs_add_executable(navigation_coordinator src/navigation_coordinator.cpp)
#cs_add_executable(lin_steering_wrt_odom src/lin_steering_wrt_odom.cpp)
cs_add_executable(lin_steering_wrt_amcl_and_odom src/lin_steering_wrt_amcl_and_odom.cpp)
cs_add_executable(locomotion_action_server src/locomotion_action_server.cpp)


#the following is required, if desire to link a node in this package with a library created in this same package
# edit the arguments to reference the named node and named library within this package
target_link_libraries(des_state_publisher_service traj_builder)
target_link_libraries(navigation_coordinator traj_builder)
target_link_libraries(locomotion_action_server traj_builder)

cs_install()
cs_export()