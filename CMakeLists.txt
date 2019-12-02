cmake_minimum_required(VERSION 2.8.3)
project(audio_transporter)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wno-narrowing")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
)

add_message_files(
   FILES
   Audio.msg
)

generate_messages(
   DEPENDENCIES
   std_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs message_runtime
)

INCLUDE(FindPkgConfig)
pkg_check_modules (JACK jack)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${JACK_INCLUDE_DIRS}
)

find_package(jack)

add_executable(sender src/sender.cpp)
add_executable(player src/player.cpp)

add_dependencies(sender ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(player ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(sender ${JACK_LIBRARIES} ${catkin_LIBRARIES})
target_link_libraries(player ${JACK_LIBRARIES} ${catkin_LIBRARIES})


