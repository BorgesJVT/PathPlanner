cmake_minimum_required(VERSION 3.1)
project(path_planner)

set(SRC_FILES 
    ${CMAKE_CURRENT_SOURCE_DIR}/src/a_star.cpp
)
add_executable(${PROJECT_NAME} ${SRC_FILES})