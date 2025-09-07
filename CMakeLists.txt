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

# New: Coverage
option(ENABLE_COVERAGE    "Enable gcov/lcov coverage instrumentation"         OFF)

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
  elseif (MSVC)
    add_compile_options(/W4)
  endif()
endif()

# Optional sanitizers in Debug (auto-disabled if ENABLE_COVERAGE=ON)
if(ENABLE_SANITIZERS AND CMAKE_BUILD_TYPE STREQUAL "Debug" AND NOT ENABLE_COVERAGE)
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
# Coverage instrumentation (GCC/Clang + gcov)
# ---------------------------------
if(ENABLE_COVERAGE)
  if (NOT CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
    message(FATAL_ERROR "ENABLE_COVERAGE currently supports GCC/Clang only.")
  endif()

  # Coverage-friendly flags
  add_compile_options(--coverage -O0 -g)
  add_link_options(--coverage)

  # Sanitizers skew coverage; hard-disable
  if(ENABLE_SANITIZERS)
    message(STATUS "Disabling sanitizers because ENABLE_COVERAGE=ON")
    set(ENABLE_SANITIZERS OFF CACHE BOOL "" FORCE)
  endif()

  # Tools
  find_program(LCOV_EXEC lcov)
  find_program(GENHTML_EXEC genhtml)

  if(NOT LCOV_EXEC OR NOT GENHTML_EXEC)
    message(WARNING "lcov/genhtml not found. 'coverage' targets will be unavailable.")
  else()
    # Where to store reports
    set(COVERAGE_INFO         ${CMAKE_BINARY_DIR}/coverage.info)
    set(COVERAGE_INFO_CLEANED ${CMAKE_BINARY_DIR}/coverage.cleaned.info)
    set(COVERAGE_HTML_DIR     ${CMAKE_BINARY_DIR}/coverage-report)

    # Reset counters
    add_custom_target(coverage_reset
      COMMAND ${LCOV_EXEC} --directory ${CMAKE_BINARY_DIR} --zerocounters
      COMMENT "Coverage: zeroing counters in ${CMAKE_BINARY_DIR}"
    )

    # Capture raw data (does not run tests)
    add_custom_target(coverage_capture
      COMMAND ${LCOV_EXEC} --capture --directory ${CMAKE_BINARY_DIR} --output-file ${COVERAGE_INFO}
      COMMENT "Coverage: capturing raw data to ${COVERAGE_INFO}"
    )

    # Generate HTML from a cleaned .info (excludes tests, deps, system headers, app demos)
    add_custom_target(coverage_html
      COMMAND ${LCOV_EXEC} --remove ${COVERAGE_INFO}
              "*/tests/*" "*/_deps/*" "/usr/*" "*/googletest/*" "*/apps/*"
              --output-file ${COVERAGE_INFO_CLEANED}
      COMMAND ${GENHTML_EXEC} ${COVERAGE_INFO_CLEANED} --output-directory ${COVERAGE_HTML_DIR}
      COMMENT "Coverage: generating HTML report at ${COVERAGE_HTML_DIR}/index.html"
      DEPENDS coverage_capture
    )

  endif()
endif()

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

add_executable(demo_floating_boxes apps/demo_floating_boxes.cpp)
target_link_libraries(demo_floating_boxes PRIVATE engine)
apply_no_inlining(demo_floating_boxes)

add_executable(demo_repulsive_particles apps/demo_repulsive_particles.cpp)
target_link_libraries(demo_repulsive_particles PRIVATE engine)
apply_no_inlining(demo_repulsive_particles)


add_executable(demo_cloth apps/demo_cloth.cpp)
target_link_libraries(demo_cloth PRIVATE engine)
apply_no_inlining(demo_cloth)

# ---------------------------------
# Profiler convenience targets (Callgrind)
# ---------------------------------
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
  set(FETCHCONTENT_UPDATES_DISCONNECTED ON CACHE BOOL "" FORCE)

  cmake_policy(SET CMP0135 NEW)
  set(FETCHCONTENT_QUIET OFF)
  set(FETCHCONTENT_UPDATES_DISCONNECTED ON CACHE BOOL "" FORCE)
  set(FETCHCONTENT_SOURCE_DIR_GOOGLETEST "" CACHE PATH "" FORCE)
  set(FETCHCONTENT_BASE_DIR "${CMAKE_BINARY_DIR}/_deps")

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
# ---------------------------------
# Unified "coverage" target (after tests exist)
# ---------------------------------
if(ENABLE_COVERAGE)
  # Make sure lcov/genhtml are found here as well (in case of scope issues)
  find_program(LCOV_EXEC lcov)
  find_program(GENHTML_EXEC genhtml)

  if(NOT LCOV_EXEC OR NOT GENHTML_EXEC)
    message(WARNING "lcov/genhtml not found; 'coverage' target will print a helpful error.")
  endif()

  # Paths used by the coverage steps (reuse if already set)
  if(NOT DEFINED COVERAGE_INFO)
    set(COVERAGE_INFO         ${CMAKE_BINARY_DIR}/coverage.info)
  endif()
  if(NOT DEFINED COVERAGE_INFO_CLEANED)
    set(COVERAGE_INFO_CLEANED ${CMAKE_BINARY_DIR}/coverage.cleaned.info)
  endif()
  if(NOT DEFINED COVERAGE_HTML_DIR)
    set(COVERAGE_HTML_DIR     ${CMAKE_BINARY_DIR}/coverage-report)
  endif()

  # Define a top-level "coverage" target that runs: reset -> ctest -> capture -> clean -> genhtml
  
  add_custom_target(coverage
    COMMAND ${CMAKE_COMMAND} -E echo "[Coverage] zero counters"
    COMMAND ${LCOV_EXEC} --directory ${CMAKE_BINARY_DIR} --zerocounters

    COMMAND ${CMAKE_COMMAND} -E echo "[Coverage] run tests"
    COMMAND ${CMAKE_COMMAND} -E env CTEST_OUTPUT_ON_FAILURE=1
            ${CMAKE_CTEST_COMMAND} --test-dir ${CMAKE_BINARY_DIR} --output-on-failure

    COMMAND ${CMAKE_COMMAND} -E echo "[Coverage] capture raw data to ${COVERAGE_INFO}"
    COMMAND ${LCOV_EXEC} --capture --directory ${CMAKE_BINARY_DIR} --output-file ${COVERAGE_INFO}

    COMMAND ${CMAKE_COMMAND} -E echo "[Coverage] filter to ${COVERAGE_INFO_CLEANED}"
    COMMAND ${LCOV_EXEC} --remove ${COVERAGE_INFO}
        "*/tests/*" "*/_deps/*" "/usr/*" "*/googletest/*" "*/apps/*" "*/c++/12/*"
        --output-file ${COVERAGE_INFO_CLEANED}
 

    COMMAND ${CMAKE_COMMAND} -E make_directory ${COVERAGE_HTML_DIR}
    COMMAND ${CMAKE_COMMAND} -E echo "[Coverage] generate HTML at ${COVERAGE_HTML_DIR}/index.html"
    COMMAND ${GENHTML_EXEC} ${COVERAGE_INFO_CLEANED} --output-directory ${COVERAGE_HTML_DIR}

    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
    DEPENDS tests
    COMMENT "Full coverage run"
  )

endif()