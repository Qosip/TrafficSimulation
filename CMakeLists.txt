cmake_minimum_required(VERSION 3.25)
project(TrafficSimulation)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(SFML 2.5 COMPONENTS graphics window system REQUIRED)

file(GLOB_RECURSE SOURCES "src/*.cpp")

add_executable(TrafficSimulation main.cpp ${SOURCES}
        src/IAgent.hpp
        src/World.cpp
        src/Camera.cpp
        src/Perception.cpp
        src/Intersection.cpp
)

target_include_directories(TrafficSimulation PRIVATE src/)

target_link_libraries(TrafficSimulation PRIVATE
        sfml-graphics
        sfml-window
        sfml-system
)

if(EXISTS ${CMAKE_SOURCE_DIR}/assets)
    add_custom_command(TARGET TrafficSimulation POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_directory
            ${CMAKE_SOURCE_DIR}/assets $<TARGET_FILE_DIR:TrafficSimulation>/assets
    )
endif()