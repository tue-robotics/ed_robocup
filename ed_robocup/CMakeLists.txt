cmake_minimum_required(VERSION 2.8.3)
project(ed_robocup)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    cb_planner_msgs_srvs
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
add_dependencies(ed_robocup_plugin ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

