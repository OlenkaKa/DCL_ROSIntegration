# Include the directory itself as a path to include directories
SET(CMAKE_INCLUDE_CURRENT_DIR ON)

function(USE_ROS)
    if (NOT DEFINED ENV{ROS_DISTRO})
        MESSAGE(FATAL_ERROR "ROS not sourced")
    else ()
        SET(ROS_DISTRO $ENV{ROS_DISTRO})
        MESSAGE(STATUS "ROS distro: ${ROS_DISTRO}")
    endif ()

    # Cannot use ARGN directly with list() command.
    # Copy to a variable first.
    set(extra_macro_args ${ARGN})

    # Did we get any optional args?
    list(LENGTH extra_macro_args num_extra_args)
    if (${num_extra_args} GREATER 0)
        MESSAGE("Using ROS packages:")
        foreach (loop_var ${extra_macro_args})
            message("${loop_var}")
            FIND_PACKAGE(${loop_var})
            set(INT_ROS_LIBRARIES ${INT_ROS_LIBRARIES} ${${loop_var}_LIBRARIES})
        endforeach ()
    endif ()

    set(ROS_LIBRARIES ${ROS_LIBRARIES} ${INT_ROS_LIBRARIES} PARENT_SCOPE)
endfunction()

USE_ROS(object_recognition_msgs roscpp std_msgs tf geometry_msgs tf_conversions object_recognition_msgs eigen_conversions)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        object_recognition_msgs
        tf
        )
include_directories(${catkin_INCLUDE_DIRS})
# Create a variable containing all .cpp files:
FILE(GLOB files *.cpp)

# Find required packages
#FIND_PACKAGE( OpenCV REQUIRED )


# Create an executable file from sources:
ADD_LIBRARY(RecognizedObjectPublisher SHARED ${files})

# Link external libraries
TARGET_LINK_LIBRARIES(RecognizedObjectPublisher ${DisCODe_LIBRARIES}
        ${ROS_LIBRARIES} ${catkin_LIBRARIES} ${object_recognition_msgs_LIBRARIES})

INSTALL_COMPONENT(RecognizedObjectPublisher)
