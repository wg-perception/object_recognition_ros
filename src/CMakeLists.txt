add_subdirectory(info_cache)
add_subdirectory(info_service)
add_subdirectory(io)

# generate ecto cells to wrap the different messages
pubsub_gen_wrap(object_recognition_msgs DESTINATION object_recognition_ros/ecto_cells
                                        INSTALL
                                        MESSAGES ObjectType ObjectInformation RecognizedObject RecognizedObjectArray Table TableArray
)
