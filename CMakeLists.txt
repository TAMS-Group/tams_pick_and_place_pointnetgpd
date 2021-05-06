cmake_minimum_required(VERSION 3.0.2)
project(tams_pick_and_place_pointnetgpd)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
        moveit_ros_planning_interface
        moveit_core
        roscpp
        std_msgs
        geometry_msgs
        sensor_msgs
        gpd_grasp_msgs
        eigen_conversions
        tf
        pcl_ros
        message_filters
        message_generation
        )

catkin_package(
        CATKIN_DEPENDS message_runtime
)

include_directories(
        ${catkin_INCLUDE_DIRS}
)

add_executable(pick_object_demo src/pick_object_demo.cpp)
target_link_libraries(pick_object_demo ${catkin_LIBRARIES})
add_dependencies(pick_object_demo ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(plan_grasps_service src/plan_grasps_service.cpp)
target_link_libraries(plan_grasps_service ${catkin_LIBRARIES})
add_dependencies(plan_grasps_service ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
