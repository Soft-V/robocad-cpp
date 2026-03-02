cmake_minimum_required (VERSION 3.8)

project ("robocad-cpp" VERSION 1.3.0)

include_directories("include")
include_directories("include/internal")
include_directories("include/internal/common")
add_library (robocad-cpp SHARED "src/studica.cpp" "src/internal/studica_internal.cpp" "src/internal/common/robot.cpp" )

if (CMAKE_VERSION VERSION_GREATER 3.12)
  set_property(TARGET robocad-cpp PROPERTY CXX_STANDARD 20)
endif()
