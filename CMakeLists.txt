cmake_minimum_required(VERSION 3.25)
project(TrafficSimulation)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ---------------------------------------------------------
# 1. CONFIGURATION SFML (Adaptée pour Linux/WSL)
# ---------------------------------------------------------
# On force la version dynamique (fichiers .so)
set(SFML_STATIC OFF CACHE BOOL "" FORCE)
# On donne le chemin exact pour éviter que CMake ne se perde
set(SFML_DIR "/usr/lib/x86_64-linux-gnu/cmake/SFML" CACHE PATH "" FORCE)

find_package(SFML 2.5 COMPONENTS graphics window system REQUIRED)

# ---------------------------------------------------------
# 2. DÉPENDANCES IMGUI (Versions stables verrouillées)
# ---------------------------------------------------------
include(FetchContent)

# A. Téléchargement de Dear ImGui (Version stable v1.88)
FetchContent_Declare(
        imgui
        GIT_REPOSITORY https://github.com/ocornut/imgui.git
        GIT_TAG v1.88
)
FetchContent_GetProperties(imgui)
if(NOT imgui_POPULATED)
    FetchContent_Populate(imgui)
endif()

# B. Téléchargement de ImGui-SFML (Version stable v2.5)
FetchContent_Declare(
        imgui-sfml
        GIT_REPOSITORY https://github.com/SFML/imgui-sfml.git
        GIT_TAG v2.5
)

# C. Configuration des chemins internes pour éviter les crashs
# On indique à ImGui-SFML où se trouve le dossier source d'ImGui
set(IMGUI_DIR ${imgui_SOURCE_DIR} CACHE PATH "Path to ImGui headers" FORCE)
# On interdit à ImGui-SFML de chercher SFML de son côté pour éviter les conflits
set(IMGUI_SFML_FIND_SFML OFF CACHE BOOL "Disable internal SFML search" FORCE)

FetchContent_MakeAvailable(imgui-sfml)

# ---------------------------------------------------------
# 3. COMPILATION DE L'EXÉCUTABLE
# ---------------------------------------------------------
file(GLOB_RECURSE SOURCES "src/*.cpp")

add_executable(TrafficSimulation
        main.cpp
        ${SOURCES}
)

# ---------------------------------------------------------
# 4. INCLUSIONS ET ÉDITION DES LIENS
# ---------------------------------------------------------
target_include_directories(TrafficSimulation PRIVATE src/)

target_link_libraries(TrafficSimulation PRIVATE
        sfml-graphics
        sfml-window
        sfml-system
        ImGui-SFML::ImGui-SFML
)

# ---------------------------------------------------------
# 5. POST-BUILD : COPIE DES ASSETS
# ---------------------------------------------------------
if(EXISTS ${CMAKE_SOURCE_DIR}/assets)
    add_custom_command(TARGET TrafficSimulation POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_directory
            ${CMAKE_SOURCE_DIR}/assets $<TARGET_FILE_DIR:TrafficSimulation>/assets
    )
endif()