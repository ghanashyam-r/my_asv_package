cmake_minimum_required(VERSION 2.8.3)

project(ASV_CATAM_URDF_ASSEMBLY)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
  roslib
  roslaunch
  robot_localization
)

catkin_package()

find_package(roslaunch)

foreach(dir config launch meshes urdf)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
add_executable(gps_waypoint src/gps_waypoint.cpp)
add_executable(collect_gps_waypoints src/collect_gps_waypoints.cpp)
target_link_libraries(gps_waypoint ${catkin_LIBRARIES})
target_link_libraries(collect_gps_waypoints ${catkin_LIBRARIES})