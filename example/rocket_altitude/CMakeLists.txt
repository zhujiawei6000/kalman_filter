cmake_minimum_required(VERSION 3.15)

add_executable(rocket_altitude_example main.cpp)
target_link_libraries(rocket_altitude_example PUBLIC ${CMAKE_PROJECT_NAME})