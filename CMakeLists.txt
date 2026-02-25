cmake_minimum_required(VERSION 3.8)
project(prometheus)

find_package(ament_cmake REQUIRED)

install(
    DIRECTORY
        config
        launch
        models
        params
        rviz
        worlds
    DESTINATION
        share/${PROJECT_NAME}/
)

install(PROGRAMS
    scripts/pink_ik_node.py
    scripts/pink_ik_test_poses.py
    DESTINATION lib/${PROJECT_NAME}
)


ament_package()
