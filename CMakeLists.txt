add_executable(gap_follow src/gap_follow.cpp)
ament_target_dependencies(my_node rclcpp)
install(TARGETS gap_follow
    DESTINATION lib/${PROJECT_NAME})
