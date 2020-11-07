project(particle_filter)

cmake_minimum_required(VERSION 3.5)

add_definitions(-std=c++11)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS "${CXX_FLAGS}")

file(GLOB HEADERS src/*.h)
file(GLOB HEADERS_HPP src/*.hpp)

set(sources src/particle_filter.cpp src/main.cpp ${HEADERS} ${HEADERS_HPP})

add_executable(${PROJECT_NAME} ${sources})


target_link_libraries(${PROJECT_NAME} z ssl uv uWS)

