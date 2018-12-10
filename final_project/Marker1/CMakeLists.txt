cmake_minimum_required(VERSION 3.12)
project(FinalProject_Image1)

# Enable C++11
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find external modules
find_package(OpenCV 3 REQUIRED)

# Compile options
if ((CMAKE_CXX_COMPILER_ID MATCHES "Clang") OR (CMAKE_CXX_COMPILER_ID MATCHES "GNU"))
    add_compile_options(-Wall -Wextra -Wpedantic -Wshadow -Wcast-qual -Wcast-align)
endif ()

# Target executable
add_executable(FinalProject_Image1 opencv_hough.cpp extract_points.h)
target_link_libraries(FinalProject_Image1 ${OpenCV_LIBS})
