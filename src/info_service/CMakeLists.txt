add_executable(object_information_server info_service.cpp)
target_link_libraries(object_information_server ${catkin_LIBRARIES}
                                                object_information_cache
)

install(TARGETS object_information_server
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
