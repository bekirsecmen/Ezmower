cmake_minimum_required(VERSION 3.5)
project(ezmower_sim)

find_package(ament_cmake REQUIRED)

install(
    DIRECTORY config launch
    DESTINATION share/${PROJECT_NAME}
)


install(
    DIRECTORY worlds/
    DESTINATION share/${PROJECT_NAME}/worlds
)

install(
    DIRECTORY models/
    DESTINATION share/${PROJECT_NAME}/models
)

ament_package()
