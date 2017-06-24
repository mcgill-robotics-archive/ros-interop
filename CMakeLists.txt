cmake_minimum_required(VERSION 2.8.3)
project(interop)

find_package(catkin REQUIRED COMPONENTS
  genmsg
  message_generation
  rospy
  sensor_msgs
  std_msgs
  visualization_msgs
)

# Python src libraries setup.
catkin_python_setup()

# Custom ROS messages.
add_message_files(FILES
  Color.msg
  FlyZone.msg
  FlyZoneArray.msg
  Orientation.msg
  Shape.msg
  Target.msg
  TargetType.msg
  UTMZone.msg
)

# Custom ROS services.
add_service_files(FILES
  AddTarget.srv
  DeleteTarget.srv
  DeleteTargetImage.srv
  GetAllTargets.srv
  GetMissionByID.srv
  GetTarget.srv
  GetTargetCompressedImage.srv
  GetTargetImage.srv
  SetTargetCompressedImage.srv
  SetTargetImage.srv
  UpdateTarget.srv
)

generate_messages(DEPENDENCIES sensor_msgs std_msgs)

catkin_package(
  CATKIN_DEPENDS
  message_generation
  message_runtime
  rospy
  sensor_msgs
  std_msgs
  visualization_msgs
)

install(
  PROGRAMS
  scripts/missions_client.py
  scripts/obstacles_client.py
  scripts/targets_server.py
  scripts/telemetry_client.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest(test/serializers.test)
  add_rostest(test/client.test)
  add_rostest(test/local_targets.test)
endif()
