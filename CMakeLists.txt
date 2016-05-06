cmake_minimum_required(VERSION 2.8.3)
project(interop)

find_package(catkin REQUIRED COMPONENTS
  rospy
  genmsg
  std_msgs
  sensor_msgs
  visualization_msgs
  message_generation
)

# Python src libraries setup.
catkin_python_setup()

# Custom ROS messages.
add_message_files(FILES
  # Enumeration types.
  Color.msg
  Orientation.msg
  Shape.msg
  TargetType.msg
  
  # Target.
  Target.msg
)

# Custom ROS services.
add_service_files(FILES
  # Target services.
  AddTarget.srv
  GetTarget.srv
  UpdateTarget.srv
  DeleteTarget.srv
  GetAllTargets.srv

  # Target image services.
  SetTargetImage.srv
  GetTargetImage.srv
  DeleteTargetImage.srv
)

generate_messages(DEPENDENCIES std_msgs sensor_msgs)

catkin_package(
  CATKIN_DEPENDS
  rospy
)
