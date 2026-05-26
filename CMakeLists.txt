cmake_minimum_required(VERSION 3.25)
project(TrafficSimulation CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ---------------------------------------------------------
# 0. OPTIONS
# ---------------------------------------------------------
option(TRAFFIC_BUILD_TESTS "Build the GoogleTest suite" ON)

# ---------------------------------------------------------
# 1. SFML (cross-platform)
#    Sous Linux/WSL on prend la version systeme si dispo,
#    sinon laisse find_package se debrouiller (vcpkg, etc.).
# ---------------------------------------------------------
set(SFML_STATIC OFF CACHE BOOL "" FORCE)

if(UNIX AND NOT APPLE AND EXISTS "/usr/lib/x86_64-linux-gnu/cmake/SFML")
    set(SFML_DIR "/usr/lib/x86_64-linux-gnu/cmake/SFML" CACHE PATH "" FORCE)
endif()

find_package(SFML 2.5 COMPONENTS graphics window system REQUIRED)

# Threads (std::thread du banc d'essai Monte-Carlo) : -pthread sous GCC/Clang.
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

# ---------------------------------------------------------
# 2. DEAR IMGUI + IMGUI-SFML (FetchContent)
# ---------------------------------------------------------
include(FetchContent)

FetchContent_Declare(
        imgui
        GIT_REPOSITORY https://github.com/ocornut/imgui.git
        GIT_TAG v1.88
)
FetchContent_GetProperties(imgui)
if(NOT imgui_POPULATED)
    FetchContent_Populate(imgui)
endif()

FetchContent_Declare(
        imgui-sfml
        GIT_REPOSITORY https://github.com/SFML/imgui-sfml.git
        GIT_TAG v2.5
)
set(IMGUI_DIR ${imgui_SOURCE_DIR} CACHE PATH "Path to ImGui headers" FORCE)
set(IMGUI_SFML_FIND_SFML OFF CACHE BOOL "Disable internal SFML search" FORCE)
FetchContent_MakeAvailable(imgui-sfml)

# ---------------------------------------------------------
# 3. CIBLE : simcore (bibliotheque statique)
#    Contient toute la logique simulation + rendu actuel.
#    Etape 0 du refactor : structure 3 cibles, code inchange.
# ---------------------------------------------------------
file(GLOB_RECURSE SIMCORE_SOURCES CONFIGURE_DEPENDS "src/*.cpp")

add_library(simcore STATIC ${SIMCORE_SOURCES})
target_include_directories(simcore PUBLIC src/)
target_link_libraries(simcore PUBLIC
        sfml-graphics
        sfml-window
        sfml-system
        Threads::Threads
)

# ---------------------------------------------------------
# 4. CIBLE : simapp (executable principal)
# ---------------------------------------------------------
add_executable(TrafficSimulation main.cpp)
target_link_libraries(TrafficSimulation PRIVATE
        simcore
        ImGui-SFML::ImGui-SFML
)

# Post-build : assets
if(EXISTS ${CMAKE_SOURCE_DIR}/assets)
    add_custom_command(TARGET TrafficSimulation POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_directory
            ${CMAKE_SOURCE_DIR}/assets $<TARGET_FILE_DIR:TrafficSimulation>/assets
    )
endif()

# ---------------------------------------------------------
# 5. CIBLE : simtests (GoogleTest, optionnelle)
# ---------------------------------------------------------
if(TRAFFIC_BUILD_TESTS)
    FetchContent_Declare(
            googletest
            GIT_REPOSITORY https://github.com/google/googletest.git
            GIT_TAG v1.14.0
    )
    # Empeche conflit avec runtime MSVC / partage des CRT
    set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(googletest)

    enable_testing()
    include(GoogleTest)

    file(GLOB SIMTEST_SOURCES "tests/test_*.cpp")
    if(SIMTEST_SOURCES)
        add_executable(simtests ${SIMTEST_SOURCES})
        target_link_libraries(simtests PRIVATE
                simcore
                GTest::gtest_main
        )
        target_compile_definitions(simtests PRIVATE
                BASELINE_PATH="${CMAKE_SOURCE_DIR}/tests/baseline.csv"
        )
        gtest_discover_tests(simtests)
    endif()
endif()
