cmake_minimum_required(VERSION 3.0.2)
project(bluerov2_exp)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mavros_msgs
  roscpp
  std_msgs
  image_transport
  sensor_msgs
  message_generation
  genmsg
  visualization_msgs
  rospy
  cv_bridge
)

find_package(OpenCV REQUIRED)
find_package(apriltag)


# catkin_python_setup()

catkin_package(
#  INCLUDE_DIRS include
  CATKIN_DEPENDS rospy geometry_msgs mavros_msgs roscpp std_msgs message_generation
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${apriltag_INCLUDE_DIRS}
)

add_executable(offb_a
    src/offb_a.cpp
)

target_link_libraries(offb_a
   ${catkin_LIBRARIES}
)

add_executable(offb_b
    src/offb_b.cpp
)

target_link_libraries(offb_b
   ${catkin_LIBRARIES}
)

add_executable(offb_c
    src/offb_c.cpp
)

target_link_libraries(offb_c
   ${catkin_LIBRARIES}
)


add_executable(cam_node
   src/cam.cpp
)
add_dependencies(cam_node
   ${${PROJECT_NAME}_EXPORTED_TARGETS} 
   ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(
  cam_node
  ${catkin_LIBRARIES} 
  ${OpenCV_LIBRARIES}
)


add_executable(odom
   src/odom.cpp
)
add_dependencies(odom
   ${${PROJECT_NAME}_EXPORTED_TARGETS} 
   ${catkin_EXPORTED_TARGETS}
   ${apriltag_EXPORTED_TARGETS}

)
target_link_libraries(
  odom
  ${catkin_LIBRARIES} 
  ${OpenCV_LIBRARIES}
  apriltag
)


add_executable(odom_mocap
   src/odom_mocap.cpp
)
add_dependencies(odom_mocap
   ${${PROJECT_NAME}_EXPORTED_TARGETS} 
   ${catkin_EXPORTED_TARGETS}
   ${apriltag_EXPORTED_TARGETS}

)
target_link_libraries(
  odom_mocap
  ${catkin_LIBRARIES} 
  ${OpenCV_LIBRARIES}
  apriltag
)

add_executable(calib
   src/calib.cpp
)
add_dependencies(calib
   ${${PROJECT_NAME}_EXPORTED_TARGETS} 
   ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(
  calib
  ${catkin_LIBRARIES} 
  ${OpenCV_LIBRARIES}
)

