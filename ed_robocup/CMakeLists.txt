cmake_minimum_required(VERSION 3.0.2)
project(ed_robocup)

add_compile_options(-Wall -Werror=all)
add_compile_options(-Wextra -Werror=extra)

find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  cb_base_navigation_msgs
  ed
  ${PROJECT_NAME}_msgs
  ed_sensor_integration
  head_ref_msgs
  nav_msgs
  rgbd
  rgbd_image_buffer
  rgbd_msgs
)

# ------------------------------------------------------------------------------------------------
#                                          CATKIN EXPORT
# ------------------------------------------------------------------------------------------------

catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}_plugin
#  CATKIN_DEPENDS
#  DEPENDS
)

# ------------------------------------------------------------------------------------------------
#                                              BUILD
# ------------------------------------------------------------------------------------------------

include_directories(
  SYSTEM
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}_plugin
  src/map_filter.cpp
  src/navigator.cpp
  src/plugin.cpp
  src/visualizer.cpp
)
target_link_libraries(${PROJECT_NAME}_plugin ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME}_plugin ${catkin_EXPORTED_TARGETS})

# ------------------------------------------------------------------------------------------------
#                                            INSTALL
# ------------------------------------------------------------------------------------------------

install(
  FILES plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
  TARGETS ${PROJECT_NAME}_plugin
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

# ------------------------------------------------------------------------------------------------
#                                              TESTS
# ------------------------------------------------------------------------------------------------
if (CATKIN_ENABLE_TESTING)
  find_package(catkin_lint_cmake REQUIRED)
  catkin_add_catkin_lint_test(-W2)
endif()
