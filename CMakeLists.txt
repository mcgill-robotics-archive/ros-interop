cmake_minimum_required(VERSION 2.8.3)
project(interop)

find_package(catkin REQUIRED COMPONENTS
  rospy
  genmsg
  std_msgs
  nav_msgs
  sensor_msgs
  geometry_msgs
)

catkin_python_setup()

add_message_files(FILES
  # Enumeration types.
  Color.msg
  Orientation.msg
  Shape.msg
  TargetType.msg
  
  # Target.
  Target.msg)
generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  CATKIN_DEPENDS
  rospy
)
