cmake_minimum_required(VERSION 2.8)
project(object_recognition_ros)

find_package(catkin REQUIRED cmake_modules ecto ecto_ros geometric_shapes geometry_msgs object_recognition_core
                             object_recognition_msgs pluginlib rosbag roscpp sensor_msgs
                             visualization_msgs
)
catkin_package(INCLUDE_DIRS include
  LIBRARIES object_information_cache
  CATKIN_DEPENDS object_recognition_core object_recognition_msgs
)

#install targets for all things python
catkin_python_setup()

# build some cells
include_directories(SYSTEM ${catkin_INCLUDE_DIRS})
include_directories(include)

add_subdirectory(python)

#set the default path for built libraries to the lib directory
if (NOT CMAKE_LIBRARY_OUTPUT_DIRECTORY)
  set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
endif()

find_package(Boost REQUIRED system filesystem serialization)

add_subdirectory(src)

# add tests
if(CATKIN_ENABLE_TESTING)
  find_package(rostest)
  add_subdirectory(test)
endif()

# add docs
catkin_doxygen(object_recognition_ros-doxygen ${CMAKE_CURRENT_SOURCE_DIR}/include)
ecto_sphinx(${CMAKE_CURRENT_SOURCE_DIR}/doc/source doc)

# install the executables
install(PROGRAMS apps/server apps/client
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# install the configuration files
install(DIRECTORY ${PROJECT_SOURCE_DIR}/conf
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# install the include dirs
install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
