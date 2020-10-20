cmake_minimum_required(VERSION 3.0.2)
project(serial_controllers)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  serial
  std_msgs
  message_generation
)

add_message_files(FILES
Test.msg
GPS.msg
STM32_control.msg
Pulse.msg
AGV_Back.msg
)

 add_service_files(
  FILES
  IMU_CMD.srv
 )

generate_messages(
DEPENDENCIES
std_msgs
)



catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES serial_controllers
  CATKIN_DEPENDS roscpp rospy serial std_msgs message_runtime
#  DEPENDS system_lib
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(GPS_serial src/GPS_serial.cpp)
target_link_libraries(GPS_serial ${catkin_LIBRARIES})
add_dependencies(GPS_serial ${catkin_EXPORTED_TARGETS})

add_executable(Test_serial src/Test_serial.cpp)
target_link_libraries(Test_serial ${catkin_LIBRARIES})
add_dependencies(Test_serial ${catkin_EXPORTED_TARGETS})

add_executable(STM_serial src/STM_serial.cpp)
target_link_libraries(STM_serial ${catkin_LIBRARIES})
add_dependencies(STM_serial ${catkin_EXPORTED_TARGETS})

add_executable(GPS_Deal src/GPS_Deal.cpp)
target_link_libraries(GPS_Deal ${catkin_LIBRARIES})
add_dependencies(GPS_Deal ${catkin_EXPORTED_TARGETS})

add_executable(IMU_serial src/IMU_serial.cpp)
target_link_libraries(IMU_serial ${catkin_LIBRARIES})
add_dependencies(IMU_serial ${catkin_EXPORTED_TARGETS})

add_executable(Test src/Test.cpp)
target_link_libraries(Test ${catkin_LIBRARIES})
add_dependencies(Test ${catkin_EXPORTED_TARGETS})

add_executable(GPS_Test src/GPS_Test.cpp)
target_link_libraries(GPS_Test ${catkin_LIBRARIES})
add_dependencies(GPS_Test ${catkin_EXPORTED_TARGETS})

#add_executable(GPS_serial src/GPS_serial.cpp)
#target_link_libraries(GPS_serial ${catkin_LIBRARIES})
#add_dependencies(GPS_serial ${catkin_EXPORTED_TARGETS})

#add_executable(Test_serial src/Test_serial.cpp)
#target_link_libraries(Test_serial ${catkin_LIBRARIES})
#add_dependencies(Test_serial ${catkin_EXPORTED_TARGETS})

#add_executable(STM_serial src/STM_serial.cpp)
#target_link_libraries(STM_serial ${catkin_LIBRARIES})
#add_dependencies(STM_serial ${catkin_EXPORTED_TARGETS})

#add_executable(GPS_Deal src/GPS_Deal.cpp)
#target_link_libraries(GPS_Deal ${catkin_LIBRARIES})
#add_dependencies(GPS_Deal ${catkin_EXPORTED_TARGETS})

#add_executable(IMU_serial src/IMU_serial.cpp)
#target_link_libraries(IMU_serial ${catkin_LIBRARIES})
#add_dependencies(IMU_serial ${catkin_EXPORTED_TARGETS})

#add_executable(Test src/Test.cpp)
#target_link_libraries(Test ${catkin_LIBRARIES})
#add_dependencies(Test ${catkin_EXPORTED_TARGETS})

#add_executable(GPS_Test src/GPS_Test.cpp)
#target_link_libraries(GPS_Test ${catkin_LIBRARIES})
#add_dependencies(GPS_Test ${catkin_EXPORTED_TARGETS})


