cmake_minimum_required(VERSION 2.8.3)
project(interop)

find_package(catkin REQUIRED COMPONENTS
  genmsg
  geographic_msgs
  message_generation
  rospy
  sensor_msgs
  std_msgs
  geometry_msgs
)

# Python src libraries setup.
catkin_python_setup()

# Custom ROS messages.
add_message_files(FILES
  Color.msg
  FlyZone.msg
  FlyZoneArray.msg
  GeoCylinder.msg
  GeoCylinderArrayStamped.msg
  GeoPolygon.msg
  GeoPolygonStamped.msg
  GeoSphere.msg
  GeoSphereArrayStamped.msg
  Object.msg
  ObjectType.msg
  Orientation.msg
  Shape.msg
  WayPoints.msg
)

# Custom ROS services.
add_service_files(FILES
  AddObject.srv
  DeleteObject.srv
  DeleteObjectImage.srv
  GetAllObjects.srv
  GetMissionByID.srv
  GetObject.srv
  GetObjectCompressedImage.srv
  GetObjectImage.srv
  SetObjectCompressedImage.srv
  SetObjectImage.srv
  UpdateObject.srv
)

generate_messages(DEPENDENCIES geographic_msgs sensor_msgs std_msgs geometry_msgs)

catkin_package(
  CATKIN_DEPENDS
  geographic_msgs
  message_generation
  message_runtime
  rospy
  sensor_msgs
  std_msgs
  geometry_msgs
)

install(
  PROGRAMS
  scripts/missions_client.py
  scripts/objects_server.py
  scripts/obstacles_client.py
  scripts/telemetry_client.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest(test/serializers.test)
  add_rostest(test/client.test)
  add_rostest(test/local_objects.test)
endif()
