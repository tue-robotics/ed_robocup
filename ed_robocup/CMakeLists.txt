cmake_minimum_required(VERSION 3.0.2)
project(ed_robocup)

## Compile as C++14, supported in ROS Melodic and newer
add_compile_options(-std=c++14)

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

add_library(ed_robocup_plugin
    src/plugin.cpp
    src/map_filter.cpp
    src/visualizer.cpp
    src/navigator.cpp
)
target_link_libraries(ed_robocup_plugin ${catkin_LIBRARIES})
add_dependencies(ed_robocup_plugin ${catkin_EXPORTED_TARGETS})

