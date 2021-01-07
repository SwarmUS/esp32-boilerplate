message("-- Building for catkin env")
find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    message_generation
    )

#TODO: Uncomment once messages are defined
#add_message_files(
#    DIRECTORY ros/msg
#    FILES
#    ${ROS_MESSAGES_LIST}
#)

#generate_messages(
#        DEPENDENCIES
#        std_msgs
#)

#TODO: Uncomment if service_generation is needed
# add_service_files(
#     DIRECTORY ros/srv
#     FILES
#     ${ROS_SERVICES_LIST}
# )



catkin_package(CATKIN_DEPENDS message_runtime std_msgs)
