cmake_minimum_required(VERSION 2.8.3)
project(mission_planner)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  tf
  mavros_msgs
  mg_msgs
  px4_control
  mav_trajectory_generation_ros
  p4_ros
  mapper
)

find_package(Eigen REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  DEPENDS Eigen
  mg_msgs
  mapper
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

# Simulation inspection node
add_executable(inspector_node 
               src/nodes/mission_node.cpp
               src/inspection_class.cpp
               src/mission_class.cpp
               src/drone_threads.cpp
               src/helper.cpp
               src/callbacks.cpp
               src/visualization_functions.cpp)
target_link_libraries(inspector_node ${catkin_LIBRARIES})
add_dependencies(inspector_node mavros_msgs_generate_messages_cpp mg_msgs_gencpp
                 mapper_gencpp mav_trajectory_generation_ros_gencpp ${catkin_EXPORTED_TARGETS}
                 ${mg_msgs_EXPORTED_TARGETS})

# Rover inspection node
add_executable(vicon_rover_inspection 
               src/nodes/mission_node.cpp
               src/vicon_rover_inspection.cpp
               src/mission_class.cpp
               src/drone_threads.cpp
               src/helper.cpp
               src/callbacks.cpp
               src/visualization_functions.cpp)
target_link_libraries(vicon_rover_inspection ${catkin_LIBRARIES})
add_dependencies(vicon_rover_inspection mavros_msgs_generate_messages_cpp mg_msgs_gencpp
                 mapper_gencpp mav_trajectory_generation_ros_gencpp ${catkin_EXPORTED_TARGETS}
                 ${mg_msgs_EXPORTED_TARGETS})

# Mapper inspection node
add_executable(mapper_inspection 
               src/nodes/mapper_node.cpp
               src/mapper_inspection_class.cpp
               src/mission_class.cpp
               src/drone_threads.cpp
               src/helper.cpp
               src/callbacks.cpp
               src/visualization_functions.cpp)
target_link_libraries(mapper_inspection ${catkin_LIBRARIES})
add_dependencies(mapper_inspection mavros_msgs_generate_messages_cpp mg_msgs_gencpp
                 mapper_gencpp mav_trajectory_generation_ros_gencpp ${catkin_EXPORTED_TARGETS}
                 ${mg_msgs_EXPORTED_TARGETS})

# Swarm inspection node
add_executable(vicon_swarm_inspection 
               src/nodes/swarm_mission_node.cpp
               src/vicon_swarm_inspection.cpp
               src/mission_class.cpp
               src/drone_threads.cpp
               src/rover_mission_class.cpp
               src/rover_threads.cpp
               src/helper.cpp
               src/callbacks.cpp
               src/polynomials.cpp
               src/visualization_functions.cpp)
target_link_libraries(vicon_swarm_inspection ${catkin_LIBRARIES})
add_dependencies(vicon_swarm_inspection mavros_msgs_generate_messages_cpp mg_msgs_gencpp
                 mapper_gencpp mav_trajectory_generation_ros_gencpp ${catkin_EXPORTED_TARGETS}
                 ${mg_msgs_EXPORTED_TARGETS})

# Collision Avoidance nodes
add_executable(collision_avoidance_fixed_map 
               src/nodes/collision_avoidance_fixed_map_node.cpp
               src/collision_avoidance_fixed_map.cpp
               src/mission_class.cpp
               src/drone_threads.cpp
               src/helper.cpp
               src/callbacks.cpp
               src/polynomials.cpp
               src/visualization_functions.cpp)
target_link_libraries(collision_avoidance_fixed_map ${catkin_LIBRARIES})
add_dependencies(collision_avoidance_fixed_map mavros_msgs_generate_messages_cpp mg_msgs_gencpp
                 mapper_gencpp mav_trajectory_generation_ros_gencpp ${catkin_EXPORTED_TARGETS}
                 ${mg_msgs_EXPORTED_TARGETS})

add_executable(collision_avoidance
               src/nodes/collision_avoidance_node.cpp
               src/collision_avoidance.cpp
               src/mission_class.cpp
               src/drone_threads.cpp
               src/helper.cpp
               src/callbacks.cpp
               src/polynomials.cpp
               src/visualization_functions.cpp)
target_link_libraries(collision_avoidance ${catkin_LIBRARIES})
add_dependencies(collision_avoidance mavros_msgs_generate_messages_cpp mg_msgs_gencpp
                 mapper_gencpp mav_trajectory_generation_ros_gencpp ${catkin_EXPORTED_TARGETS}
                 ${mg_msgs_EXPORTED_TARGETS})
