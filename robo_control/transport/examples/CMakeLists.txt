# Joint Command Publisher Example
add_executable(publisher_joint_command
    publisher_joint_command.cpp
)
target_link_libraries(publisher_joint_command
    PRIVATE
        transport
        idl_generated
)


# Joint Command Subscriber Example
add_executable(subscriber_joint_command
    subscriber_joint_command.cpp
)
target_link_libraries(subscriber_joint_command
    PRIVATE
        transport
        idl_generated
)
