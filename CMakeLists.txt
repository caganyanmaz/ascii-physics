cmake_minimum_required(VERSION 3.20)
project(ascii_physics LANGUAGES CXX)

# Compiler setup
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Global options
option(ENABLE_CUDA "Enable GPU backend (CUDA)" OFF)
option(ENABLE_TESTS "Enable building tests" ON)

# Include dirs
include_directories(${CMAKE_SOURCE_DIR}/include)

# ---------------------------
# Engine library (core code)
# ---------------------------
file(GLOB_RECURSE ENGINE_HEADERS ${CMAKE_SOURCE_DIR}/include/engine/*.hpp)
file(GLOB_RECURSE ENGINE_SOURCES ${CMAKE_SOURCE_DIR}/src/*.cpp)

add_library(engine ${ENGINE_HEADERS} ${ENGINE_SOURCES})
target_include_directories(engine PUBLIC ${CMAKE_SOURCE_DIR}/include)

# If CUDA backend is later enabled:
if(ENABLE_CUDA)
    enable_language(CUDA)
    file(GLOB_RECURSE CUDA_SOURCES ${CMAKE_SOURCE_DIR}/backends/gpu_backend/*.cu)
    add_library(cuda_backend ${CUDA_SOURCES})
    target_link_libraries(engine PUBLIC cuda_backend)
    target_compile_definitions(engine PUBLIC HAVE_CUDA_BACKEND=1)
endif()

# ---------------------------
# Demo executable
# ---------------------------
add_executable(demo_ascii apps/demo_ascii.cpp)
target_link_libraries(demo_ascii PRIVATE engine)

# ---------------------------
# Tests (GoogleTest)
# ---------------------------
if(ENABLE_TESTS)
    include(FetchContent)
    FetchContent_Declare(
      googletest
      URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.zip
    )
    # prevent GoogleTest from overriding compile options
    set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(googletest)

    # Collect test sources
    file(GLOB_RECURSE TEST_SOURCES ${CMAKE_SOURCE_DIR}/tests/*.cpp)

    add_executable(tests ${TEST_SOURCES})
    target_link_libraries(tests PRIVATE engine GTest::gtest_main)

    include(GoogleTest)
    gtest_discover_tests(tests)
endif()

