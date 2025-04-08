cmake_minimum_required(VERSION 3.8)
project(beckhoff_testbed_description)

find_package(ament_cmake REQUIRED)

install(
    DIRECTORY
        config
        launch
        urdf
    DESTINATION
        share/${PROJECT_NAME}
)

install(PROGRAMS
        DESTINATION lib/${PROJECT_NAME})

ament_package()
