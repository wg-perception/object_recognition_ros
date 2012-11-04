# check that at least the help completes correctly
#object_recognition_core_help_test(${CMAKE_CURRENT_SOURCE_DIR}/../src/server/server.py)
object_recognition_core_help_test(${CMAKE_CURRENT_SOURCE_DIR}/../src/server/client.py)

# check the different sources
object_recognition_core_source_test(bag_reader "object_recognition_ros.io" "{'bag':'a.bag'}")
object_recognition_core_source_test(ros_kinect "object_recognition_ros.io" "{}")

# check the different sinks
object_recognition_core_sink_test(publisher "object_recognition_ros.io" "{}")

find_package(object_recognition_tod QUIET)
if (object_recognition_tod_FOUND)
  object_recognition_core_detection_test(${CMAKE_CURRENT_SOURCE_DIR}/../src/server/conf/config_detection.sample)
endif()
