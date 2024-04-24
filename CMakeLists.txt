cmake_minimum_required(VERSION 3.8)
project(leap_hand_densetact)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


## Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
#find_package(cv_bridge REQUIRED)
#find_package(OpenCV REQUIRED)
#find_package(orocos_kdl REQUIRED)
#find_package(kdl_parser REQUIRED)
#find_package(message_generation REQUIRED)

#set(opencv_DIR "/home/mod-dt/opencv_build/opencv/build")

#set(kdl_DIR "/home/armlab/leaphand_ws/src/orocos_kinematics_dynamics/orocos_kdl")

#add_executable(dt_video scripts/cpp_dt_video.cpp)
#ament_target_dependencies(dt_video rclcpp std_msgs OpenCV cv_bridge)

#add_executable(ee_forces scripts/end_effector_forces.cpp)
#ament_target_dependencies(ee_forces rclcpp std_msgs geometry_msgs sensor_msgs orocos_kdl kdl_parser)

install(PROGRAMS
    scripts/leaphand_node.py

    
    DESTINATION lib/${PROJECT_NAME}
)
#install(TARGETS
#    dt_video
#    ee_forces
#    DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch #model
        DESTINATION share/${PROJECT_NAME})

################################################
## Declare ROS messages, services and actions ##
################################################


find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/LeapVelocity.srv"
  "srv/LeapPosition.srv"
  "srv/LeapEffort.srv"
#  "srv/KinovaCollision.srv"
 )
 
 ament_python_install_package(leap_hand_utils)

#add_service_files(
#  FILES
#  LeapVelocity.srv
#  LeapPosition.srv
#  LeapEffort.srv
#)

#generate_messages(
#  DEPENDENCIES
#  std_msgs
#)

###########
## Build ##
###########

include_directories(
  include
  ${ament_INCLUDE_DIRS}
)

ament_export_dependencies(
  std_msgs
  message_runtime
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

#############
## Testing ##
#############

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

