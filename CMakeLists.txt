cmake_minimum_required(VERSION 3.0.2)
project(RobotDesign3_poop)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
)

catkin_package()

link_directories(
  ${catkin_LIBRARY_DIRS}
)

install(DIRECTORY launch worlds
  DESTINATION $(CATKIN_PACKAGE_SHARE_DESTINATION)
)
