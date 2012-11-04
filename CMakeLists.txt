cmake_minimum_required(VERSION 2.8)
project(object_recognition_ros)

find_package(catkin REQUIRED ecto ecto_ros object_recognition_core)
if (${catkin_VERSION} VERSION_GREATER "0.5.28")
catkin_package()
else()
catkin_stack()
catkin_project(object_recognition_ros)
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
