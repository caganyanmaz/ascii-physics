cmake_minimum_required(VERSION 3.20)
project(ascii_physics LANGUAGES CXX)

# ---------------------------------
# Compiler setup
# ---------------------------------
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ---------------------------------
# Global options
# ---------------------------------
option(ENABLE_CUDA "Enable GPU backend (CUDA)" OFF)
option(ENABLE_TESTS "Enable building tests" ON)

# ---------------------------------
# Include dirs
# ---------------------------------
include_directories(${CMAKE_SOURCE_DIR}/include)

# ---------------------------------
# Engine library (core code)
# ---------------------------------
file(GLOB_RECURSE ENGINE_HEADERS ${CMAKE_SOURCE_DIR}/include/engine/*.hpp)
file(GLOB_RECURSE ENGINE_SOURCES ${CMAKE_SOURCE_DIR}/src/*.cpp)

add_library(engine ${ENGINE_HEADERS} ${ENGINE_SOURCES})
target_include_directories(engine PUBLIC ${CMAKE_SOURCE_DIR}/include)

# ---------------------------------
# Optional CUDA backend
# ---------------------------------
if(ENABLE_CUDA)
    enable_language(CUDA)
    file(GLOB_RECURSE CUDA_SOURCES ${CMAKE_SOURCE_DIR}/backends/gpu_backend/*.cu)
    add_library(cuda_backend ${CUDA_SOURCES})
    target_link_libraries(engine PUBLIC cuda_backend)
    target_compile_definitions(engine PUBLIC HAVE_CUDA_BACKEND=1)
endif()

# ---------------------------------
# Demo executables
# ---------------------------------
add_executable(demo_ascii apps/demo_ascii.cpp)
target_link_libraries(demo_ascii PRIVATE engine)

add_executable(demo_spring apps/demo_spring.cpp)
target_link_libraries(demo_spring PRIVATE engine)

# ---------------------------------
# Tests (GoogleTest)
# ---------------------------------
if(ENABLE_TESTS)
    include(FetchContent)
    FetchContent_Declare(
      googletest
      URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.zip
    )
    # Prevent GoogleTest from overriding CRT settings on MSVC
    set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(googletest)

    # Collect test sources
    file(GLOB_RECURSE TEST_SOURCES ${CMAKE_SOURCE_DIR}/tests/*.cpp)

    add_executable(tests ${TEST_SOURCES})
    target_link_libraries(tests PRIVATE engine GTest::gtest_main)

    include(CTest)
    include(GoogleTest)
    gtest_discover_tests(tests
      DISCOVERY_MODE PRE_TEST
    )
    target_compile_definitions(tests PRIVATE GOLDEN_DIR="${CMAKE_SOURCE_DIR}/tests/golden")

    # ---------------------------
    # Golden snapshot support
    # ---------------------------
    # Turn this ON only when you *intend* to refresh baselines locally.
    option(UPDATE_GOLDEN "Rewrite golden snapshot files in tests (DO NOT USE IN CI)" OFF)


    # If we're rebuilding goldens, compile tests with the macro enabled.
    if(UPDATE_GOLDEN)
        target_compile_definitions(tests PRIVATE UPDATE_GOLDEN=1)
    endif()

    # Allow filtering which tests to run when updating goldens (regex over test names)
    set(GOLDEN_UPDATE_REGEX ".*Regression.*" CACHE STRING
        "ctest --tests-regex used by the 'update_golden' target (default: runs *Regression* tests)")

    # Convenience target: build tests (with UPDATE_GOLDEN if set) and run only selected tests to refresh goldens.
    add_custom_target(update_golden
        COMMAND ${CMAKE_COMMAND} -E env CTEST_OUTPUT_ON_FAILURE=1
                ${CMAKE_CTEST_COMMAND}
                --test-dir ${CMAKE_BINARY_DIR}
                --output-on-failure
                --tests-regex "${GOLDEN_UPDATE_REGEX}"
        DEPENDS tests
        COMMENT "Updating golden snapshots by running tests with UPDATE_GOLDEN=${UPDATE_GOLDEN} and regex='${GOLDEN_UPDATE_REGEX}'"
    )
endif()