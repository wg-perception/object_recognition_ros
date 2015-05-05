# check that at least the help completes correctly
object_recognition_core_help_test(${CMAKE_CURRENT_SOURCE_DIR}/../apps/server)
object_recognition_core_help_test(${CMAKE_CURRENT_SOURCE_DIR}/../apps/client)

# check the different sources
object_recognition_core_source_test(BagReader "object_recognition_ros.io" "{'bag':'a.bag'}")
object_recognition_core_source_test(RosKinect "object_recognition_ros.io" "{}")

# check the different sinks
object_recognition_core_sink_test(Publisher "object_recognition_ros.io.sink" "{}")

# check the test config file
object_recognition_core_config_test(${CMAKE_CURRENT_SOURCE_DIR}/../conf/detection.test.ros.ork)

# test the actionlib server (requires roscoreto belaunched)
add_rostest(action_server.test)
