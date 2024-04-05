cmake_minimum_required(VERSION 3.15)

add_executable(vehicle_location_example main.cpp)
target_link_libraries(vehicle_location_example PUBLIC ${CMAKE_PROJECT_NAME})