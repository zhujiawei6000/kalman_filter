cmake_minimum_required(VERSION 3.15)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
project(KalmanFilter CXX)
find_package(Eigen3 REQUIRED)
add_executable(${CMAKE_PROJECT_NAME} main.cxx)

target_link_libraries(${CMAKE_PROJECT_NAME} PUBLIC Eigen3::Eigen)
# target_include_directories(${CMAKE_PROJECT_NAME} PUBLIC ${Eigen3_INCLUDE_DIR})