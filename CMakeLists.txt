cmake_minimum_required(VERSION 2.8.3)
project(xiaoqiang_tts)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
)

find_package(PythonLibs 2 REQUIRED)

include_directories(
  ${PYTHON_INCLUDE_PATH}
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/src/engines/xunfei/sdk/src/
  ${PROJECT_SOURCE_DIR}/src/engines/xunfei/sdk/include/
  ${PROJECT_SOURCE_DIR}/src/engines/xunfei/sdk/
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS roscpp rospy sensor_msgs std_msgs
)

link_libraries(${PYTHON_LIBRARIES})

add_library(xunfei_tts
  ${PROJECT_SOURCE_DIR}/src/engines/xunfei/sdk/src/speech_recognizer.c
  ${PROJECT_SOURCE_DIR}/src/engines/xunfei/sdk/src/xunfei_asr.cpp
  ${PROJECT_SOURCE_DIR}/src/engines/xunfei/sdk/src/xunfei_tts.cpp
)

target_link_libraries(xunfei_tts
  ${catkin_LIBRARIES}
  ${PROJECT_SOURCE_DIR}/src/engines/xunfei/sdk/libs/x64/libmsc.so
)

set_target_properties(xunfei_tts PROPERTIES OUTPUT_NAME xunfei_tts PREFIX "_" SUFFIX ".so")
set_target_properties(xunfei_tts PROPERTIES COMPILE_FLAGS "-g -Wno-missing-field-initializers")
set_target_properties(xunfei_tts PROPERTIES
  ARCHIVE_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}
  LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}
)

# target_include_directories(xunfei_tts
#   ${catkin_INCLUDE_DIRS}
# )

# install(
#   TARGETS ${PROJECT_NAME} ${PROJECT_NAME}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# install(
#   DIRECTORY launch
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

# install(
#   DIRECTORY udevs
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

# install(
#   DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
# )

# install(
#   FILES  scripts/fixudev
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )