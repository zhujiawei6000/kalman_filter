cmake_minimum_required(VERSION 3.15)

add_executable(vehicle_location_with_radar main.cpp)
target_link_libraries(vehicle_location_with_radar PUBLIC ${CMAKE_PROJECT_NAME})