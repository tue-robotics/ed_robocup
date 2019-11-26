cmake_minimum_required(VERSION 2.8.3)
project(ed_robocup_msgs)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
)

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
  DEPENDENCIES std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs
)
