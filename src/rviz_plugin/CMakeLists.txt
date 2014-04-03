include_directories(SYSTEM ${catkin_INCLUDE_DIRS})

find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
include(${QT_USE_FILE})
add_definitions(-DQT_NO_KEYWORDS)

qt4_wrap_cpp(MOC_FILES
  ork_object_display.h
  ork_table_display.h
)

set(SOURCE_FILES
  ork_object_display.cpp
  ork_object_visual.cpp
  ork_table_display.cpp
  ork_table_visual.cpp
  ${MOC_FILES}
)

add_library(${PROJECT_NAME}_rviz ${SOURCE_FILES})
target_link_libraries(${PROJECT_NAME}_rviz ${catkin_LIBRARIES}
                                           object_information_cache
                                           ${QT_LIBRARIES}
)
install(TARGETS ${PROJECT_NAME}_rviz
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
