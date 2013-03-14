include_directories(${catkin_INCLUDE_DIRS})

find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
include(${QT_USE_FILE})
add_definitions(-DQT_NO_KEYWORDS)

qt4_wrap_cpp(MOC_FILES
  ork_display.h
)

set(SOURCE_FILES
  ork_display.cpp
  ork_visual.cpp
  ${MOC_FILES}
)

add_library(${PROJECT_NAME}_rviz ${SOURCE_FILES})
target_link_libraries(${PROJECT_NAME}_rviz ${catkin_LIBRARIES}
                                           object_information_cache
                                           ${QT_LIBRARIES}
)
