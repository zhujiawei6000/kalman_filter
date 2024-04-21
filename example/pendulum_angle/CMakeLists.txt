cmake_minimum_required(VERSION 3.15)

add_executable(pendulum_angle_example main.cpp)
target_link_libraries(pendulum_angle_example PUBLIC ${CMAKE_PROJECT_NAME})