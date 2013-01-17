cmake_minimum_required(VERSION 2.8)
project(object_recognition_ros)

find_package(catkin REQUIRED)

if (${catkin_VERSION} VERSION_GREATER "0.5.28")
find_package(catkin REQUIRED ecto ecto_ros geometry_msgs object_recognition_core
                             object_recognition_msgs rosbag roscpp rviz sensor_msgs tf
                             visualization_msgs
)
catkin_package()
else()
find_package(ecto)
find_package(object_recognition_core)
find_package(object_recognition_msgs)
catkin_stack()
catkin_project(object_recognition_ros)
find_package(ecto)
find_package(ecto_ros)
find_package(geometry_msgs)
find_package(object_recognition_core)
find_package(object_recognition_msgs)
find_package(rosbag)
find_package(roscpp)
find_package(sensor_msgs)
find_package(visualization_msgs)

list(APPEND catkin_LIBRARIES ${object_recognition_core_LIBRARIES} 
                             ${roscpp_LIBRARIES}
                             ${geometry_msgs_LIBRARIES}
                             ${sensor_msgs_LIBRARIES}
                             ${rosbag_LIBRARIES}
                             ${roscpp_LIBRARIES}
#                             /opt/ros/fuerte/stacks/geometry/tf/lib/libtf.so
)
list(APPEND catkin_INCLUDE_DIRS ${roscpp_INCLUDE_DIRS}
#                                /opt/ros/fuerte/stacks/geometry/tf/include/
#                                /opt/ros/fuerte/stacks/bullet/include/
#                                /opt/ros/fuerte/stacks/geometry/tf/msg_gen/cpp/include/
#                                /opt/ros/fuerte/stacks/geometry/tf/srv_gen/cpp/include/
#                                /opt/ros/fuerte/stacks/visualization/rviz/src/
#                                /opt/ros/fuerte/stacks/pluginlib/include/
)
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

#catkin_doxygen(object_recognition_ros-doxygen ${CMAKE_CURRENT_SOURCE_DIR}/include)

# install
if (ROS_FUERTE_FOUND)
set(CATKIN_PACKAGE_SHARE_DESTINATION share/${PROJECT_NAME}/)
else()
install(FILES ork_rviz_plugin.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
endif()
