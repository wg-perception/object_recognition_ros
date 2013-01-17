# check that at least the help completes correctly
#object_recognition_core_help_test(${CMAKE_CURRENT_SOURCE_DIR}/../src/server/server.py)
object_recognition_core_help_test(${CMAKE_CURRENT_SOURCE_DIR}/../src/server/client.py)

# check the different sources
object_recognition_core_source_test(BagReader "object_recognition_ros.io" "{'bag':'a.bag'}")
object_recognition_core_source_test(RosKinect "object_recognition_ros.io" "{}")

# check the different sinks
object_recognition_core_sink_test(Publisher "object_recognition_ros.io.sink" "{}")

find_package(object_recognition_tod QUIET)
if (object_recognition_tod_FOUND)
  object_recognition_core_config_test(${CMAKE_CURRENT_SOURCE_DIR}/../src/server/conf/config_detection.sample)
endif()
