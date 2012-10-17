if (ROS_GROOVY_OR_ABOVE_FOUND)
include(${CATKIN_BUILD_PREFIX}/${CATKIN_GLOBAL_SHARE_DESTINATION}/object_recognition_core/cmake/test.cmake)
else()
include(${CMAKE_BINARY_DIR}/cmake/object_recognition_core/test.cmake)
endif()

# check that at least the help completes correctly
#object_recognition_core_help_test(${CMAKE_CURRENT_SOURCE_DIR}/../src/server/server.py)
object_recognition_core_help_test(${CMAKE_CURRENT_SOURCE_DIR}/../src/server/client.py)

# check the different sources
object_recognition_core_source_test(bag_reader "{'bag':'a.bag'}" "object_recognition_core.io")
object_recognition_core_source_test(ros_kinect "{}" "object_recognition_core.io")

# check the different sinks
object_recognition_core_sink_test(publisher "{}" "object_recognition_core.io")
