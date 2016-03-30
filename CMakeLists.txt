cmake_minimum_required(VERSION 2.8.3)
project(ed_robocup)

find_package(catkin REQUIRED COMPONENTS
    ed_sensor_integration
    ed_perception
    ed
    rgbd
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
)
target_link_libraries(ed_robocup_plugin ${catkin_LIBRARIES})
add_dependencies(ed_robocup_plugin ${PROJECT_NAME}_generate_messages_cpp)

