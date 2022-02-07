cmake_minimum_required(VERSION 3.0.2)
project(kinematic_library)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp
#  DEPENDS other non-ROS libs
)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
add_library(${PROJECT_NAME}
  src/EndEffector.cpp
  src/MyUrd5.cpp
  src/Position.cpp
  src/RPYRotation.cpp
  src/ur5Kinematic.cpp
  src/Ur5DH.cpp
  src/Ur5JointConfiguration.cpp
  src/Callback_jointconfig.cpp
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
