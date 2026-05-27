cmake_minimum_required(VERSION 3.25)
project(TrafficSimulation CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ---------------------------------------------------------
# 0. OPTIONS
# ---------------------------------------------------------
option(TRAFFIC_BUILD_TESTS "Build the GoogleTest suite" ON)
# La CIBLE GUI (Dear ImGui + ImGui-SFML + OpenGL) n'est utile que pour jouer la
# simulation. Les jobs CI "tests" et "couverture" la desactivent : ils n'ont
# besoin que de simcore (logique) + simtests/simbench (headless), ce qui evite
# de tirer ImGui/OpenGL sur les runners.
option(TRAFFIC_BUILD_APP "Build the interactive GUI executable" ON)
# Instrumentation gcov/lcov (GCC/Clang) pour le job 'metrics' de la CI.
option(TRAFFIC_COVERAGE  "Enable gcov coverage instrumentation"  OFF)

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
# 2. DEAR IMGUI + IMGUI-SFML (FetchContent) -- GUI uniquement
# ---------------------------------------------------------
include(FetchContent)

if(TRAFFIC_BUILD_APP)
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
endif()

# ---------------------------------------------------------
# 2bis. COUVERTURE (gcov) : helper applique aux cibles ciblees
# ---------------------------------------------------------
function(traffic_enable_coverage target)
    if(TRAFFIC_COVERAGE AND NOT MSVC)
        target_compile_options(${target} PRIVATE --coverage -O0 -g)
        target_link_options(${target}    PRIVATE --coverage)
    endif()
endfunction()

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
traffic_enable_coverage(simcore)

# ---------------------------------------------------------
# 4. CIBLE : simapp (executable principal, GUI)
# ---------------------------------------------------------
if(TRAFFIC_BUILD_APP)
    add_executable(TrafficSimulation main.cpp)
    target_link_libraries(TrafficSimulation PRIVATE
            simcore
            ImGui-SFML::ImGui-SFML
    )

    # Pas de fenetre console au lancement du .exe : sous-systeme WINDOWS (GUI).
    # On conserve le point d'entree standard main() (pas de WinMain) via
    # /ENTRY:mainCRTStartup -> aucune modification du code source necessaire.
    if(WIN32)
        set_target_properties(TrafficSimulation PROPERTIES WIN32_EXECUTABLE TRUE)
        if(MSVC)
            target_link_options(TrafficSimulation PRIVATE /ENTRY:mainCRTStartup)
        endif()
    endif()

    # Post-build : assets
    if(EXISTS ${CMAKE_SOURCE_DIR}/assets)
        add_custom_command(TARGET TrafficSimulation POST_BUILD
                COMMAND ${CMAKE_COMMAND} -E copy_directory
                ${CMAKE_SOURCE_DIR}/assets $<TARGET_FILE_DIR:TrafficSimulation>/assets
        )
    endif()
endif()

# ---------------------------------------------------------
# 4bis. CIBLE : simbench (benchmark de performance headless)
#       Aucune dependance GUI -> toujours construit (utile en CI et en local).
# ---------------------------------------------------------
add_executable(simbench tools/benchmark.cpp)
target_link_libraries(simbench PRIVATE simcore)
traffic_enable_coverage(simbench)

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
        traffic_enable_coverage(simtests)
        gtest_discover_tests(simtests)
    endif()
endif()
