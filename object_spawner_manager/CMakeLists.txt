cmake_minimum_required(VERSION 3.8)
project(object_spawner_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(rclpy REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(urdfdom REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(spawn_object_interfaces REQUIRED)

find_package(std_msgs REQUIRED)

message(STATUS "urdf_INCLUDE_DIRS: ${urdf_INCLUDE_DIRS}")

# uncomment the following section in order to fill in further dependencies manually.
# find_package(<dependency> REQUIRED)

# Include Cpp "include" directory
include_directories(
  ${moveit_core_INCLUDE_DIRS}
  ${moveit_msgs_INCLUDE_DIRS}
  ${moveit_INCLUDE_DIRS}
  ${urdf_INCLUDE_DIRS}  
  ${urdfdom_INCLUDE_DIRS}  
  ${moveit_ros_planning_interface_INCLUDE_DIRS}
  ${shapes_INCLUDE_DIRS}
  include
)
# Create Cpp executable
add_executable(spawn_object_in_moveit src/spawn_object_in_moveit.cpp)

ament_target_dependencies(spawn_object_in_moveit 
  rclcpp 
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  geometric_shapes
  ${THIS_PACKAGE_INCLUDE_DEPENDS} 
  Boost 
  moveit_msgs
  rclcpp_action
  tf2_ros
  std_msgs
  spawn_object_interfaces
)

target_link_libraries(spawn_object_in_moveit
  ${DEPENDENCIES}
)

# Create Cpp executable
add_executable(spawn_object_in_moveit_V src/spawn_object_in_moveit_V.cpp)

ament_target_dependencies(spawn_object_in_moveit_V
  rclcpp 
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  geometric_shapes
  ${THIS_PACKAGE_INCLUDE_DEPENDS} 
  Boost 
  moveit_msgs
  rclcpp_action
  tf2_ros
  std_msgs
  spawn_object_interfaces
)

target_link_libraries(spawn_object_in_moveit_V
  ${DEPENDENCIES}
)

# Install Cpp executables
install(TARGETS
  spawn_object_in_moveit
  spawn_object_in_moveit_V
  DESTINATION lib/${PROJECT_NAME}
)
# Install Python modules
ament_python_install_package(${PROJECT_NAME})
# Install Python executables

install(PROGRAMS
  object_spawner_manager/spawn_object_tf.py
  object_spawner_manager/object_spawner_manager.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
