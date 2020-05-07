cmake_minimum_required(VERSION 2.8.3)
project(rosgraph_monitor)

find_package(catkin REQUIRED COMPONENTS
    rospy
    std_msgs
    message_generation
)

catkin_python_setup()

add_service_files(
   FILES
   PredictAction.srv
)

generate_messages(
    DEPENDENCIES
    std_msgs
)

catkin_package(
   CATKIN_DEPENDS message_runtime
)

install(DIRECTORY launch
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

