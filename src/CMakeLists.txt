add_subdirectory(io)
add_subdirectory(rviz_plugin)

# generate ecto cells to wrap the different messages
pubsub_gen_wrap(object_recognition_msgs DESTINATION object_recognition_ros/ecto_cells
                                        INSTALL
                                        MESSAGES ObjectType ObjectInformation RecognizedObject RecognizedObjectArray Table TableArray
)
