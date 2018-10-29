cmake_minimum_required(VERSION 3.1)
project(image4_2)

# Enable C++11
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find external modules
find_package(OpenCV 3 REQUIRED)

# Compile options
if((CMAKE_CXX_COMPILER_ID MATCHES "Clang") OR (CMAKE_CXX_COMPILER_ID MATCHES "GNU"))
  add_compile_options(-Wall -Wextra -Wpedantic -Wshadow -Wcast-qual -Wcast-align)
endif()

# Target executable
add_executable(image4_2 src/image4_2.cpp)
target_link_libraries(image4_2 ${OpenCV_LIBS})
