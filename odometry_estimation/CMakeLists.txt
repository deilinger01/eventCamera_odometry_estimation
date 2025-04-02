cmake_minimum_required(VERSION 3.0.2)
project(odometry_estimation)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  event_msgs  # Add event_msgs to the required components
  message_generation
)

# Generate messages if you have custom messages
# Uncomment and modify this section if you plan to add your own messages
# add_message_files(
#   FILES
#   YourMessage.msg
# )

# generate_messages(
#   DEPENDENCIES
#   std_msgs
#   sensor_msgs
# )

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs event_msgs message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# Add your source files here if you have a node to compile
# Uncomment and modify this section if you plan to add a node
# add_executable(${PROJECT_NAME}_node src/your_node.cpp)
# target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})
