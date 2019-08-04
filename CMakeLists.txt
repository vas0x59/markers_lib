cmake_minimum_required(VERSION 2.8.12)

project(markers_lib)

if(MINGW)
    add_definitions("-D_USE_MATH_DEFINES") # For M_PI
endif()

if(MSVC)
    add_definitions("-std=c++11 -WX -W2")
    add_definitions("-D_USE_MATH_DEFINES") # For M_PI
else()
    add_definitions("-std=c++11 -Wall -Wextra -Werror")
endif()

find_package( OpenCV REQUIRED )



add_library(markers_lib SHARED
markers_lib/aruco_markers.cpp
markers_lib/solver.cpp
markers_lib/markers.cpp
)

install(
    TARGETS markers
    LIBRARY DESTINATION /usr/lib
)
