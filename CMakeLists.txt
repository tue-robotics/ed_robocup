cmake_minimum_required(VERSION 2.8.3)
project(ed_robocup)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    cb_planner_msgs_srvs
    ed
    ed_sensor_integration
    head_ref
    nav_msgs
    rgbd
    rgbd_msgs
)

# find_package(Boost REQUIRED COMPONENTS system program_options)
# find_package(PCL REQUIRED)
# find_package(OpenCV REQUIRED)

# ------------------------------------------------------------------------------------------------
#                                     ROS MESSAGES AND SERVICES
# ------------------------------------------------------------------------------------------------

# Generate messages
add_message_files(
   FILES
   NamedImage.msg
)

# Generate services
add_service_files(
   FILES
   FitEntityInImage.srv
   GetModelImages.srv
)

# Generate added messages and services with any dependencies listed here
generate_messages(
#    DEPENDENCIES
#    geometry_msgs
#    ...
)

# ------------------------------------------------------------------------------------------------
#                                          CATKIN EXPORT
# ------------------------------------------------------------------------------------------------

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES bla
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

# ------------------------------------------------------------------------------------------------
#                                              BUILD
# ------------------------------------------------------------------------------------------------

include_directories(
    include
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

