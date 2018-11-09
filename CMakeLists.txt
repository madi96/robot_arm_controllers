cmake_minimum_required(VERSION 2.8.3)
project(robot_arm_controllers)

# Set variables
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -fPIC")
set(DEFAULT_ROBOT "pr2")
set(PRIMITIVE_SRC_FILES src/primitive.cpp)
set(PUSH_PRIMITIVE_SRC_FILES src/push_primitive.cpp)
set(LIFT_PRIMITIVE_SRC_FILES src/lift_primitive.cpp)
set(BUTTON_PRIMITIVE_SRC_FILES src/button_primitive.cpp)
set(JOYSTICK_PRIMITIVE_SRC_FILES src/joystick_primitive.cpp)

option(ROBOT "Used to specify the targeted robot" ${DEFAULT_ROBOT})
add_definitions(-DROBOT="${ROBOT}")

# Find required packages
find_package(catkin REQUIRED COMPONENTS
  moveit_ros_planning_interface
  roscpp
  rospy
  std_msgs
  tf
  trajectory_msgs
  message_generation
)

if (${ROBOT} STREQUAL "baxter")
	find_package(catkin REQUIRED COMPONENTS baxter_core_msgs)
endif()




## Generate services in the 'srv' folder
# add_service_files(
#         FILES
#         service.srv	
# )

# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )

catkin_package(
  INCLUDE_DIRS include/
  CATKIN_DEPENDS moveit_ros_planning_interface roscpp rospy std_msgs tf trajectory_msgs baxter_core_msgs message_runtime
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Mark cpp header files for installation
install(DIRECTORY include/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

add_executable (primitive ${PRIMITIVE_SRC_FILES})
target_link_libraries(primitive ${catkin_LIBRARIES})
#~ add_executable (push_primitive ${PUSH_PRIMITIVE_SRC_FILES})
#~ target_link_libraries(primitive ${catkin_LIBRARIES})
#~ add_executable (lift_primitive ${LIFT_PRIMITIVE_SRC_FILES})
#~ target_link_libraries(primitive ${catkin_LIBRARIES})
#~ add_executable (button_primitive ${BUTTON_PRIMITIVE_SRC_FILES})
#~ target_link_libraries(primitive ${catkin_LIBRARIES})
#~ add_executable (joystick_primitive ${JOYSTICK_PRIMITIVE_SRC_FILES})
#~ target_link_libraries(primitive ${catkin_LIBRARIES})




