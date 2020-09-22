cmake_minimum_required(VERSION 3.0.2)
project(four_wheel_steering_controller)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

set(${PROJECT_NAME}_CATKIN_DEPS
    controller_interface
    nav_msgs
    realtime_tools
    tf
)



find_package(catkin REQUIRED COMPONENTS ${${PROJECT_NAME}_CATKIN_DEPS} )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS ${${PROJECT_NAME}_CATKIN_DEPS}
)

include_directories(
  include ${catkin_INCLUDE_DIRS} 
)

add_library(${PROJECT_NAME} src/four_wheel_steering_controller.cpp src/odometry.cpp src/speed_limiter.cpp  src/gather_to_file.cpp

# src/ros_controller_csi_interface.cpp
)


target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${SOURCE_FILES} ${CUDA_LIBRARIES})

install(DIRECTORY include/${PROJECT_NAME} 
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

# Install library
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
  )

#install(FILES #four_wheel_steering_controller_plugins.xml
 #   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  find_package(catkin REQUIRED COMPONENTS rosgraph_msgs rostest std_srvs controller_manager tf )
  include_directories( ${catkin_INCLUDE_DIRS} )

  add_executable(ros_controller_main src/ros_controller_main.cpp)
  add_dependencies( ros_controller_main ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  target_link_libraries(ros_controller_main ${PROJECT_NAME} ${SOURCE_FILES} ${CUDA_LIBRARIES})
  add_dependencies(tests ros_controller_main)



endif()
