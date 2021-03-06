cmake_minimum_required(VERSION 2.8.12)

project(markers_lib)

# if(MINGW)
#     add_definitions("-D_USE_MATH_DEFINES") # For M_PI
# endif()

# if(MSVC)
#     add_definitions("-std=c++11 -WX -W2")
#     add_definitions("-D_USE_MATH_DEFINES") # For M_PI
# else()
#     add_definitions("-std=c++11 -Wall -Wextra -Werror")
# endif()

add_definitions("-std=gnu++11")
add_definitions("-D_USE_MATH_DEFINES")

find_package( OpenCV REQUIRED )
include_directories(${PROJECT_SOURCE_DIR}/src)


add_library(markers_lib SHARED
src/aruco_markers.cpp
src/solver.cpp
src/markers.cpp
)

target_link_libraries(markers_lib
    ${OpenCV_LIBS}
)
install(
    TARGETS markers_lib
    LIBRARY DESTINATION /usr/lib
)

add_executable(test_cam EXCLUDE_FROM_ALL example/test_cam.cpp)

# target_link_libraries(led_uart_cpp
#     pthread
# )

target_link_libraries(test_cam
    ${OpenCV_LIBS}
    pthread
    ${PROJECT_SOURCE_DIR}/${DEP2_BUILD_DIR}/libmarkers_lib.so
)