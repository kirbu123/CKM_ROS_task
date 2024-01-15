cmake_minimum_required(VERSION 2.8.3)
project(mybot_pkg)

set(CMAKE_EXPORT_COMPILE_COMMANDS  ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=address,undefined -Wall -Wextra -pedantic -Werror -Wno-self-assign-overloaded -Wno-missing-braces")

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg geometry_msgs nav_msgs)

## Declare ROS messages and services
add_message_files(FILES Num.msg)
# add_service_files(FILES AddTwoInts.srv)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs geometry_msgs nav_msgs)

## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(mybot_node src/node.cpp)
target_link_libraries(mybot_node ${catkin_LIBRARIES})
add_dependencies(mybot_node beginner_tutorials_generate_messages_cpp)

add_executable(mybot_listener src/listener.cpp)
target_link_libraries(mybot_listener ${catkin_LIBRARIES})
add_dependencies(mybot_listener beginner_tutorials_generate_messages_cpp)

add_executable(mybot_listener_upgrade src/listener_upgrade.cpp)
target_link_libraries(mybot_listener_upgrade ${catkin_LIBRARIES})
add_dependencies(mybot_listener_upgrade beginner_tutorials_generate_messages_cpp)
