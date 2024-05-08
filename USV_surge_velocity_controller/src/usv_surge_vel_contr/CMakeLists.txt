cmake_minimum_required(VERSION 3.0.2)
project(usv_surge_vel_contr)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  ras_tf_lib
)

catkin_package(

)

include_directories(

  ${catkin_INCLUDE_DIRS}
)

 catkin_install_python(PROGRAMS
   scripts/surge_vel_controller.py 
   scripts/heading_controller.py
   scripts/control_effort_allocator_nomoto.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )