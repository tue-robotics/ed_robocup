cmake_minimum_required(VERSION 3.5)
project(ed_robocup_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
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
  DEPENDENCIES
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)
