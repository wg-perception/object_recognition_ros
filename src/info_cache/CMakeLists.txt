add_library(object_information_cache info_cache.cpp)
target_link_libraries(object_information_cache ${catkin_LIBRARIES})

install(TARGETS object_information_cache
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
