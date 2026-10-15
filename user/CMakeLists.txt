target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/host_protocol.c
    ${CMAKE_CURRENT_SOURCE_DIR}/uart_dma.c
    ${CMAKE_CURRENT_SOURCE_DIR}/dm3519.c
    ${CMAKE_CURRENT_SOURCE_DIR}/hwt101ct.c
    ${CMAKE_CURRENT_SOURCE_DIR}/can_link.c
    ${CMAKE_CURRENT_SOURCE_DIR}/swerve_kinematics.c
    ${CMAKE_CURRENT_SOURCE_DIR}/motion_limits.c
    ${CMAKE_CURRENT_SOURCE_DIR}/chassis_planner.c
    ${CMAKE_CURRENT_SOURCE_DIR}/chassis_output.c
    ${CMAKE_CURRENT_SOURCE_DIR}/drive_supervisor.c
    ${CMAKE_CURRENT_SOURCE_DIR}/single_wheel_test.c
    ${CMAKE_CURRENT_SOURCE_DIR}/servo_calibration.c
    ${CMAKE_CURRENT_SOURCE_DIR}/servo_pwm.c
    ${CMAKE_CURRENT_SOURCE_DIR}/inspection_app.c
)
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
