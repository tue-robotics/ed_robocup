cmake_minimum_required(VERSION 3.0.2)
project(ed_robocup)

find_package(catkin REQUIRED COMPONENTS
    cb_base_navigation_msgs
    ed
    ed_robocup_msgs
    ed_sensor_integration
    head_ref_msgs
    nav_msgs
    rgbd
    rgbd_msgs
)

# ------------------------------------------------------------------------------------------------
#                                          CATKIN EXPORT
# ------------------------------------------------------------------------------------------------

catkin_package()

# ------------------------------------------------------------------------------------------------
#                                              BUILD
# ------------------------------------------------------------------------------------------------

include_directories(
    ${catkin_INCLUDE_DIRS}
)

add_library(ed_map_plugin
    src/map_plugin.cpp
    src/map_filter.cpp
)
target_link_libraries(ed_map_plugin ${catkin_LIBRARIES})
add_dependencies(ed_map_plugin ${catkin_EXPORTED_TARGETS})

add_library(ed_gui_plugin
    src/gui_plugin.cpp
    src/visualizer.cpp
    src/navigator.cpp
)
target_link_libraries(ed_gui_plugin ${catkin_LIBRARIES})
add_dependencies(ed_gui_plugin ${catkin_EXPORTED_TARGETS})
