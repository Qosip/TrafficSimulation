# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/mnt/c/Users/fddav/Desktop/TrafficSimulation/build-refactor/_deps/imgui-src"
  "/mnt/c/Users/fddav/Desktop/TrafficSimulation/build-refactor/_deps/imgui-build"
  "/mnt/c/Users/fddav/Desktop/TrafficSimulation/build-refactor/_deps/imgui-subbuild/imgui-populate-prefix"
  "/mnt/c/Users/fddav/Desktop/TrafficSimulation/build-refactor/_deps/imgui-subbuild/imgui-populate-prefix/tmp"
  "/mnt/c/Users/fddav/Desktop/TrafficSimulation/build-refactor/_deps/imgui-subbuild/imgui-populate-prefix/src/imgui-populate-stamp"
  "/mnt/c/Users/fddav/Desktop/TrafficSimulation/build-refactor/_deps/imgui-subbuild/imgui-populate-prefix/src"
  "/mnt/c/Users/fddav/Desktop/TrafficSimulation/build-refactor/_deps/imgui-subbuild/imgui-populate-prefix/src/imgui-populate-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/mnt/c/Users/fddav/Desktop/TrafficSimulation/build-refactor/_deps/imgui-subbuild/imgui-populate-prefix/src/imgui-populate-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/mnt/c/Users/fddav/Desktop/TrafficSimulation/build-refactor/_deps/imgui-subbuild/imgui-populate-prefix/src/imgui-populate-stamp${cfgdir}") # cfgdir has leading slash
endif()
