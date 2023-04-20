cmake_minimum_required(VERSION 3.0.2)
project(usv_formation_control_1)

#find_package(Python COMPONENTS Interpreter REQUIRED)

# install PyPI Python package using pip
#execute_process(COMMAND ${Python_EXECUTABLE} -m pip install scipy)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  rospy
  sensor_msgs
  std_msgs
  std_msgs
  tf
  tf2_ros
  geographic_msgs
  sensor_msgs
)

catkin_package(

)



include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

 catkin_install_python(PROGRAMS 
	scripts/formation_configuration_broadcaster_1.py 
	scripts/formation_state_estimator.py 
	scripts/formation_trajectory_planner.py
	scripts/vessel_reference_generator.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )