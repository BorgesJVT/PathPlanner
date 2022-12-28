cmake_minimum_required(VERSION 3.1)
project(path_planner)

set(SRC_FILES 
    ${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/map.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/navigation.cpp
)

set(HEADER_FILES 
    ${CMAKE_CURRENT_SOURCE_DIR}/include/map.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/navigation.hpp
)

include_directories(include)

add_executable(${PROJECT_NAME} ${SRC_FILES})
# target_include_directories(${PROJECT_NAME} PUBLIC ${HEADER_FILES})