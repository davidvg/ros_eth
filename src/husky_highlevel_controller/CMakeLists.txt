cmake_minimum_required(VERSION 2.8.3)
project(husky_highlevel_controller)

add_definitions(--std=c++11)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    sensor_msgs
    geometry_msgs
    visualization_msgs
    tf
)

catkin_package(
    INCLUDE_DIRS include
    #LIBRARIES
    CATKIN_DEPENDS roscpp sensor_msgs
    #DEPENDS
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}
    src/${PROJECT_NAME}_node.cpp
    src/HuskyHighlevelController.cpp
)

target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
)
