cmake_minimum_required(VERSION 2.8)
project(object_recognition_ros)

find_package(catkin REQUIRED)

if (${catkin_VERSION} VERSION_GREATER "0.5.28")
find_package(catkin REQUIRED ecto ecto_ros geometric_shapes geometry_msgs object_recognition_core
                             object_recognition_msgs pluginlib rosbag roscpp rviz sensor_msgs tf
                             visualization_msgs
)
catkin_package()
else()
find_package(ROS REQUIRED ecto ecto_ros geometry_msgs object_recognition_core
                          object_recognition_msgs rosbag roscpp sensor_msgs
                          visualization_msgs
)
catkin_stack()
catkin_project(object_recognition_ros)

set(catkin_LIBRARIES ${ROS_LIBRARIES})
set(catkin_INCLUDE_DIRS ${ROS_INCLUDE_DIRS})
set(CATKIN_DEVEL_PREFIX ${CMAKE_BINARY_DIR})
set(CATKIN_PACKAGE_BIN_DESTINATION share/${PROJECT_NAME}/bin)
set(CATKIN_PACKAGE_INCLUDE_DESTINATION include/${PROJECT_NAME})
set(CATKIN_PACKAGE_LIB_DESTINATION lib)
set(CATKIN_PACKAGE_PYTHON_BUILD_DESTINATION gen/py/${PROJECT_NAME}/)
set(CATKIN_PACKAGE_PYTHON_DESTINATION ${INSTALLED_PYTHONPATH}/${PROJECT_NAME}/)
set(CATKIN_PACKAGE_SHARE_DESTINATION share/${PROJECT_NAME}/)

add_definitions(-DROS_FUERTE_FOUND=1)
endif()

#install targets for all things python
catkin_python_setup()

# build some cells
include_directories(SYSTEM ${catkin_INCLUDE_DIRS})

add_subdirectory(python)

add_definitions("-Wno-pragmas -fno-strict-aliasing -Wall -Wl,--no-undefined -Werror")

#set the default path for built libraries to the lib directory
if (NOT CMAKE_LIBRARY_OUTPUT_DIRECTORY)
  set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
endif()

find_package(Boost REQUIRED system filesystem serialization)
find_package(OpenCV REQUIRED)

add_subdirectory(src)

# add tests
option(BUILD_TEST "Build tests" ON)
if(BUILD_TEST)
  enable_testing()
  add_subdirectory(test)
endif()

# add docs
catkin_doxygen(object_recognition_ros-doxygen ${CMAKE_CURRENT_SOURCE_DIR}/include)
ecto_sphinx(${CMAKE_CURRENT_SOURCE_DIR}/doc/source doc)

# install
if (ROS_FUERTE_FOUND)
set(CATKIN_PACKAGE_SHARE_DESTINATION share/${PROJECT_NAME}/)
else()
install(FILES ork_rviz_plugin.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
endif()

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
