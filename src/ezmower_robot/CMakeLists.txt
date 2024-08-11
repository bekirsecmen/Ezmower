cmake_minimum_required(VERSION 3.5)
project(ezmower_robot)

find_package(ament_cmake REQUIRED)

install(
    DIRECTORY config launch
    DESTINATION share/${PROJECT_NAME}
)

# write the URDF files into the build directory
install(
    DIRECTORY urdf/
    DESTINATION share/${PROJECT_NAME}/urdf
)

install(
    DIRECTORY meshes/
    DESTINATION share/${PROJECT_NAME}/meshes
)


ament_package()
