# BNO08x Example
add_executable(joystick_example
    joystick_example.cpp
)
target_link_libraries(joystick_example
    PRIVATE
        joystick_sdk
)
