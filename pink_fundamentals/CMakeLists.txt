# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(pink_fundamentals)
# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS  roscpp rospy sensor_msgs create_fundamentals std_msgs message_generation nav_msgs geometry_msgs)

# include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} )
# CATKIN_MIGRATION: removed during catkin migration
# cmake_minimum_required(VERSION 2.4.6)

# CATKIN_MIGRATION: removed during catkin migration
# include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)


# CATKIN_MIGRATION: removed during catkin migration
# rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
add_message_files(FILES
    Cell.msg
    Grid.msg
    Pose.msg
    Row.msg
    ExactPose.msg
    
)
#uncomment if you have defined services

add_service_files(
    FILES
    PID_drive.srv
    Wanderer.srv
    align_call.srv
    ExecutePlan.srv
    play_song.srv
    MoveToPosition.srv
    DistanceToNode.srv
)

#common commands for building c++ executables and libraries
#add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#
# CATKIN_MIGRATION: removed during catkin migration
# rosbuild_add_boost_directories()
#find_package(Boost REQUIRED COMPONENTS thread)

#include_directories(${Boost_INCLUDE_DIRS}  )
#target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})

#add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

## Generate added messages and services with any dependencies listed here
generate_messages(
    DEPENDENCIES 
    std_msgs 
    create_fundamentals
    geometry_msgs
    
)

# catkin_package parameters: http://ros.org/doc/groovy/api/catkin/html/dev_guide/generated_cmake_api.html#catkin-package
catkin_package(
    DEPENDS python-serial
    CATKIN_DEPENDS roscpp rospy sensor_msgs create_fundamentals message_runtime
)

include_directories(include ${catkin_INCLUDE_DIRS} include/package src)
# include_directories(include ${catkin_INCLUDE_DIRS} src/wanderer)


 add_executable(example_laser src/example_laser.cpp)
 target_link_libraries(example_laser ${catkin_LIBRARIES})
 add_dependencies(example_laser create_fundamentals_generate_messages_cpp)

 add_executable(example_moving src/example_moving.cpp)
 target_link_libraries(example_moving ${catkin_LIBRARIES})
 add_dependencies(example_moving create_fundamentals_generate_messages_cpp)

add_executable(wanderer_try_final src/wanderer_try_final.cpp)
target_link_libraries(wanderer_try_final ${catkin_LIBRARIES})
add_dependencies(wanderer_try_final create_fundamentals_generate_messages_cpp)


# add_executable(wanderer_try_left_right_rot src/wanderer_try_left_right_rot.cpp)
# target_link_libraries(wanderer_try_left_right_rot wanderer_lib ${catkin_LIBRARIES})
# add_dependencies(wanderer_try_left_right_rot create_fundamentals_generate_messages_cpp)

# add_executable(align_WandererInWhile_withIntersect src/align_WandererInWhile_withIntersect.cpp)
# target_link_libraries(align_WandererInWhile_withIntersect square_PID_lib ${catkin_LIBRARIES})
# add_dependencies(align_WandererInWhile_withIntersect create_fundamentals_generate_messages_cpp)

add_executable(PID_drive src/PID_drive.cpp)
target_link_libraries(PID_drive  wanderer_lib ${catkin_LIBRARIES})
add_dependencies(PID_drive create_fundamentals_generate_messages_cpp)

add_executable(align_server src/align_server.cpp)
target_link_libraries(align_server wanderer_lib ${catkin_LIBRARIES})
add_dependencies(align_server create_fundamentals_generate_messages_cpp)

add_library(wanderer_lib src/wanderer_try_final.cpp)
target_link_libraries(wanderer_lib ${catkin_LIBRARIES})
add_dependencies(wanderer_lib create_fundamentals_generate_messages_cpp)

#add_executable(noSongsAnymore  src/noSongsAnymore.cpp)
#target_link_libraries(noSongsAnymore  ${catkin_LIBRARIES})
#add_dependencies(noSongsAnymore  create_fundamentals_generate_messages_cpp)


add_executable(bumper_listener src/bumper_listener.cpp)
target_link_libraries(bumper_listener ${catkin_LIBRARIES})
add_dependencies(bumper_listener create_fundamentals_generate_messages_cpp)

add_executable(execute_plan_server src/execute_plan_server.cpp )
target_link_libraries(execute_plan_server wanderer_lib ${catkin_LIBRARIES})
add_dependencies(execute_plan_server create_fundamentals_generate_messages_cpp)

# add_executable(execute_ohneFancyDrive src/execute_ohneFancyDrive.cpp )
# target_link_libraries(execute_ohneFancyDrive wanderer_lib ${catkin_LIBRARIES})
# add_dependencies(execute_ohneFancyDrive create_fundamentals_generate_messages_cpp)

add_executable(potential_field src/potential_field.cpp )
target_link_libraries(potential_field  ${catkin_LIBRARIES})
add_dependencies(potential_field create_fundamentals_generate_messages_cpp)

add_executable(avf src/avf.cpp )
target_link_libraries(avf ${catkin_LIBRARIES})
add_dependencies(avf create_fundamentals_generate_messages_cpp)

add_executable(publish_map_rviz src/publish_map_rviz.cpp)
target_link_libraries(publish_map_rviz ${catkin_LIBRARIES})
add_dependencies(publish_map_rviz create_fundamentals_generate_messages_cpp)


add_executable(monte_carlo src/monte_carlo.cpp src/songs_lib.cpp)
target_link_libraries(monte_carlo ${catkin_LIBRARIES})
add_dependencies(monte_carlo create_fundamentals_generate_messages_cpp)

add_executable(monte_carlo_matrix src/monte_carlo_matrix.cpp)
target_link_libraries(monte_carlo_matrix ${catkin_LIBRARIES})
add_dependencies(monte_carlo_matrix create_fundamentals_generate_messages_cpp)

add_executable(monte_carlo_matrix_kidnap src/monte_carlo_matrix_kidnap.cpp)
target_link_libraries(monte_carlo_matrix_kidnap ${catkin_LIBRARIES})
add_dependencies(monte_carlo_matrix_kidnap create_fundamentals_generate_messages_cpp)


add_executable(avf_wanderer src/avf_wanderer.cpp)
target_link_libraries(avf_wanderer ${catkin_LIBRARIES})
add_dependencies(avf_wanderer create_fundamentals_generate_messages_cpp)

add_executable(moveToPosition_server src/moveToPosition_server.cpp src/songs_lib.cpp)
target_link_libraries(moveToPosition_server ${catkin_LIBRARIES})
add_dependencies(moveToPosition_server create_fundamentals_generate_messages_cpp)


add_executable(executeChallenge src/executeChallenge.cpp src/songs_lib.cpp)
target_link_libraries(executeChallenge ${catkin_LIBRARIES})
add_dependencies(executeChallenge create_fundamentals_generate_messages_cpp)
