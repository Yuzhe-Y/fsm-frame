cmake_minimum_required(VERSION 3.14)

project(state C CXX)

add_executable(${PROJECT_NAME})

target_include_directories(${PROJECT_NAME} include)

add_library(sml INTERFACE)
target_include_directories(sml INTERFACE ../sml/include)

target_link_libraries(${PROJECT_NAME} sml)