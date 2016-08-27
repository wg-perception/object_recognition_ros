if(("$ENV{ROS_DISTRO}" STREQUAL "indigo") OR ("$ENV{ROS_DISTRO}" STREQUAL "jade"))
else()
add_definitions(-std=c++11)
find_package(Eigen REQUIRED)
include_directories(${EIGEN_INCLUDE_DIRS})
endif()
add_library(object_information_cache info_cache.cpp)
target_link_libraries(object_information_cache ${catkin_LIBRARIES})

install(TARGETS object_information_cache
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
