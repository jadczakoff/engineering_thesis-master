cmake_minimum_required(VERSION 3.0.2)
project(six_wheel_steering_msgs)

find_package(catkin REQUIRED COMPONENTS
        std_msgs
        message_generation
        )

add_service_files(
        FILES
        ModeChange.srv
)

generate_messages(
        DEPENDENCIES
        std_msgs
)

catkin_package(
          CATKIN_DEPENDS std_msgs message_runtime
)
include_directories(

        ${catkin_INCLUDE_DIRS}
)
