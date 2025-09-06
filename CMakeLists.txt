cmake_minimum_required(VERSION 3.20)
project(ascii_physics LANGUAGES CXX)

# ---------------------------------
# Defaults & options
# ---------------------------------
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# If no build type was given, default to RelWithDebInfo (good for profiling)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "" FORCE)
endif()

option(ENABLE_CUDA        "Enable GPU backend (CUDA)"                         OFF)
option(ENABLE_TESTS       "Enable building tests"                             ON)
option(ENABLE_WARNINGS    "Enable compiler warnings"                          ON)
option(ENABLE_SANITIZERS  "Enable ASan/UBSan in Debug builds"                 ON)
option(DISABLE_INLINING   "Disable inlining to improve Callgrind call counts" OFF)

# Helpful globally
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# ---------------------------------
# Include dirs
# ---------------------------------
include_directories(${CMAKE_SOURCE_DIR}/include)

# ---------------------------------
# Platform libs (e.g., threads)
# ---------------------------------
find_package(Threads REQUIRED)

# ---------------------------------
# Build flags by config
# ---------------------------------
# Make RelWithDebInfo profiler-friendly: keep FP and avoid tail-call elision
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} -fno-omit-frame-pointer -fno-optimize-sibling-calls")

# Optional warnings
if(ENABLE_WARNINGS)
  if (CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
    add_compile_options(-Wall -Wextra -Wpedantic)
    # Keep it reasonable for now; you can add -Wconversion/-Wshadow later.
  elseif (MSVC)
    add_compile_options(/W4)
  endif()
endif()

# Optional sanitizers in Debug
if(ENABLE_SANITIZERS AND CMAKE_BUILD_TYPE STREQUAL "Debug")
  if (CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
    add_compile_options(-fsanitize=address,undefined -fno-omit-frame-pointer)
    add_link_options(-fsanitize=address,undefined)
  endif()
endif()

# Optional: disable inlining to make Callgrind’s call counts cleaner
function(apply_no_inlining target_name)
  if (DISABLE_INLINING)
    if (CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
      target_compile_options(${target_name} PRIVATE -fno-inline -fno-inline-functions -fno-inline-functions-called-once)
    elseif (MSVC)
      target_compile_options(${target_name} PRIVATE /Ob0)
    endif()
  endif()
endfunction()

# ---------------------------------
# Engine library (core code)
# ---------------------------------
file(GLOB_RECURSE ENGINE_HEADERS ${CMAKE_SOURCE_DIR}/include/engine/*.hpp)
file(GLOB_RECURSE ENGINE_SOURCES ${CMAKE_SOURCE_DIR}/src/*.cpp)

add_library(engine ${ENGINE_HEADERS} ${ENGINE_SOURCES})
target_include_directories(engine PUBLIC ${CMAKE_SOURCE_DIR}/include)
target_link_libraries(engine PUBLIC Threads::Threads)

# If you need position-independent code everywhere (shared libs later), uncomment:
# set_target_properties(engine PROPERTIES POSITION_INDEPENDENT_CODE ON)

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

# Apply no-inlining to engine if chosen
apply_no_inlining(engine)

# ---------------------------------
# Demo executables
# ---------------------------------
add_executable(demo_ascii apps/demo_ascii.cpp)
target_link_libraries(demo_ascii PRIVATE engine)
apply_no_inlining(demo_ascii)

add_executable(demo_spring apps/demo_spring.cpp)
target_link_libraries(demo_spring PRIVATE engine)
apply_no_inlining(demo_spring)

# ---------------------------------
# Profiler convenience targets (Callgrind)
# ---------------------------------
# Use --instr-atstart=no and toggle with callgrind_control for short windows.
# Reduce overhead: no cache/jumps; keep branch of interest readable.
find_program(VALGRIND_EXECUTABLE valgrind)
if(VALGRIND_EXECUTABLE)
  add_custom_target(profile_demo_ascii
    COMMAND ${VALGRIND_EXECUTABLE}
      --tool=callgrind
      --instr-atstart=no
      --collect-jumps=no
      --cache-sim=no
      --toggle-collect=Simulation::step
      $<TARGET_FILE:demo_ascii>
    DEPENDS demo_ascii
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
    COMMENT "Run demo_ascii under Callgrind (instrumentation OFF at start; toggle with callgrind_control)"
  )

  add_custom_target(profile_demo_spring
    COMMAND ${VALGRIND_EXECUTABLE}
      --tool=callgrind
      --instr-atstart=no
      --collect-jumps=no
      --cache-sim=no
      --toggle-collect=Simulation::step
      $<TARGET_FILE:demo_spring>
    DEPENDS demo_spring
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
    COMMENT "Run demo_spring under Callgrind (instrumentation OFF at start; toggle with callgrind_control)"
  )
endif()

# ---------------------------------
# Tests (GoogleTest)
# ---------------------------------
if(ENABLE_TESTS)
  include(FetchContent)
  # Speeds up repeated configs offline
  set(FETCHCONTENT_UPDATES_DISCONNECTED ON CACHE BOOL "" FORCE)

  FetchContent_Declare(
    googletest
    URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.zip
  )
  set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
  FetchContent_MakeAvailable(googletest)

  file(GLOB_RECURSE TEST_SOURCES ${CMAKE_SOURCE_DIR}/tests/*.cpp)

  add_executable(tests ${TEST_SOURCES})
  target_link_libraries(tests PRIVATE engine GTest::gtest_main)
  apply_no_inlining(tests)

  include(CTest)
  include(GoogleTest)
  gtest_discover_tests(tests
    DISCOVERY_MODE PRE_TEST
  )

  target_compile_definitions(tests PRIVATE GOLDEN_DIR="${CMAKE_SOURCE_DIR}/tests/golden")

  # Golden snapshot support
  option(UPDATE_GOLDEN "Rewrite golden snapshot files in tests (DO NOT USE IN CI)" OFF)
  if(UPDATE_GOLDEN)
    target_compile_definitions(tests PRIVATE UPDATE_GOLDEN=1)
  endif()

  set(GOLDEN_UPDATE_REGEX ".*Regression.*" CACHE STRING
      "ctest --tests-regex used by the 'update_golden' target (default: runs *Regression* tests)")

  add_custom_target(update_golden
      COMMAND ${CMAKE_COMMAND} -E env CTEST_OUTPUT_ON_FAILURE=1
              ${CMAKE_CTEST_COMMAND}
              --test-dir ${CMAKE_BINARY_DIR}
              --output-on-failure
              --tests-regex "${GOLDEN_UPDATE_REGEX}"
      DEPENDS tests
      COMMENT "Updating golden snapshots with UPDATE_GOLDEN=${UPDATE_GOLDEN} and regex='${GOLDEN_UPDATE_REGEX}'"
  )
endif()