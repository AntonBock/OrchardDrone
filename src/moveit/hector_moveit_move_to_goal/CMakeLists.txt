cmake_minimum_required(VERSION 2.8.3)
project(hector_moveit_move_to_goal)

set(CMAKE_CXX_FLAGS "-std=c++11")

find_package(catkin REQUIRED COMPONENTS 
    roscpp 
    moveit_core 
    moveit_ros_planning_interface 
    std_msgs 
    hector_uav_msgs
    hector_moveit_actions
    actionlib
    geometry_msgs
)

find_package(octomap REQUIRED)
find_package(OpenMP REQUIRED)
catkin_package(CATKIN_DEPENDS roscpp moveit_core moveit_ros_planning_interface std_msgs hector_uav_msgs hector_moveit_actions)

if(OPENMP_FOUND)
  message(STATUS "OPENMP FOUND")
  set(OpenMP_FLAGS ${OpenMP_CXX_FLAGS})
  set(OpenMP_LIBS gomp)
endif()

include_directories(include ${OCTOMAP_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

add_executable(hector_moveit_move_to_goal src/move_to_goal.cpp src/move_to_goal_node.cpp)
target_compile_options(hector_moveit_move_to_goal PRIVATE ${OpenMP_FLAGS} ${CMAKE_CXX_FLAGS})
target_link_libraries(hector_moveit_move_to_goal ${OCTOMAP_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenMP_LIBS})
add_dependencies(hector_moveit_move_to_goal hector_uav_msgs_generate_messages hector_moveit_actions_generate_messages)
