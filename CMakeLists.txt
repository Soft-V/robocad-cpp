cmake_minimum_required (VERSION 3.8)

project ("robocad-cpp" VERSION 1.3.0)

find_package(Threads REQUIRED)
find_package(OpenCV REQUIRED )

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories("include")
include_directories("include/internal")
include_directories("include/internal/common")

add_library (robocad-cpp SHARED "src/studica.cpp" "src/internal/studica_internal.cpp" "src/internal/common/robot.cpp" )

target_link_libraries(robocad-cpp PRIVATE ${OpenCV_LIBS})
target_link_libraries(robocad-cpp PRIVATE Threads::Threads)

