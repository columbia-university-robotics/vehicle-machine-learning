cmake_minimum_required(VERSION 2.8.3)
project(csi_rover_move_base)

find_package(catkin REQUIRED
    COMPONENTS
        actionlib
        base_local_planner
        clear_costmap_recovery
        cmake_modules
        costmap_2d
        dwa_local_planner
        global_planner
        dynamic_reconfigure
        geometry_msgs
        message_generation
        move_base_msgs
        nav_core
        nav_msgs
        navfn
        pluginlib
        roscpp
        rospy
        rotate_recovery
        std_srvs
        tf2_geometry_msgs
        tf2_ros
        std_msgs
)
find_package(Eigen3 REQUIRED)
add_definitions(${EIGEN3_DEFINITIONS})

catkin_package(
    CATKIN_DEPENDS
        dynamic_reconfigure
        geometry_msgs
        move_base_msgs
        nav_msgs
        roscpp
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
)

# move_base
add_library(csi_rover_move_base
  src/csi_rover_move_base.cpp
)
add_library(csi_rover_rotate_recovery
  src/csi_rover_rotate_recovery.cpp
)
target_link_libraries(csi_rover_move_base
    ${Boost_LIBRARIES}
    ${catkin_LIBRARIES}
    )
add_dependencies(csi_rover_move_base ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(csi_rover_move_base_node
  src/csi_rover_move_base_node.cpp
)
add_dependencies(csi_rover_move_base_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(csi_rover_move_base_node csi_rover_move_base)
set_target_properties(csi_rover_move_base_node PROPERTIES OUTPUT_NAME csi_rover_move_base)
install(
  FILES csi_rover_rotate_recovery.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(
    TARGETS
        csi_rover_move_base_node
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
    TARGETS
        csi_rover_move_base
    DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
