# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/mnt/c/Users/fddav/Desktop/TrafficSimulation/build-refactor/_deps/imgui-sfml-src"
  "/mnt/c/Users/fddav/Desktop/TrafficSimulation/build-refactor/_deps/imgui-sfml-build"
  "/mnt/c/Users/fddav/Desktop/TrafficSimulation/build-refactor/_deps/imgui-sfml-subbuild/imgui-sfml-populate-prefix"
  "/mnt/c/Users/fddav/Desktop/TrafficSimulation/build-refactor/_deps/imgui-sfml-subbuild/imgui-sfml-populate-prefix/tmp"
  "/mnt/c/Users/fddav/Desktop/TrafficSimulation/build-refactor/_deps/imgui-sfml-subbuild/imgui-sfml-populate-prefix/src/imgui-sfml-populate-stamp"
  "/mnt/c/Users/fddav/Desktop/TrafficSimulation/build-refactor/_deps/imgui-sfml-subbuild/imgui-sfml-populate-prefix/src"
  "/mnt/c/Users/fddav/Desktop/TrafficSimulation/build-refactor/_deps/imgui-sfml-subbuild/imgui-sfml-populate-prefix/src/imgui-sfml-populate-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/mnt/c/Users/fddav/Desktop/TrafficSimulation/build-refactor/_deps/imgui-sfml-subbuild/imgui-sfml-populate-prefix/src/imgui-sfml-populate-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/mnt/c/Users/fddav/Desktop/TrafficSimulation/build-refactor/_deps/imgui-sfml-subbuild/imgui-sfml-populate-prefix/src/imgui-sfml-populate-stamp${cfgdir}") # cfgdir has leading slash
endif()
