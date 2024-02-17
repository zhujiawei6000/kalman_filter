cmake_minimum_required(VERSION 3.15)

add_executable(example vehicle_location_example.cpp)
target_link_libraries(example PUBLIC ${CMAKE_PROJECT_NAME})