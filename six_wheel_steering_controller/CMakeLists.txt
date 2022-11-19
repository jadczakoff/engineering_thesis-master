cmake_minimum_required(VERSION 3.0.2)
project(six_wheel_steering_controller)

# Default to C++14
if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif ()

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
        controller_interface
        hardware_interface
        nav_msgs
        pluginlib
        realtime_tools
        roscpp
        tf
        urdf_geometry_parser
        six_wheel_steering_msgs
        )

find_package(Eigen3 REQUIRED)

# Declare a catkin package
catkin_package(
        INCLUDE_DIRS include
        LIBRARIES ${PROJECT_NAME}
        CATKIN_DEPENDS
        controller_interface
        hardware_interface
        nav_msgs
        realtime_tools
        roscpp
        tf
        DEPENDS EIGEN3
)


###########
## Build ##
###########

# Specify header include paths
include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
        src/six_wheel_steering_controller.cpp
        src/wheel_unit.cpp
        src/mode.cpp
        src/drive.cpp
        src/kinematics.cpp
        src/modes/mode1.cpp
        src/modes/mode2.cpp
        src/speed_limiter.cpp)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        )

# Install targets
install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
        )

# Install plugins
install(FILES six_wheel_steering_controller_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )