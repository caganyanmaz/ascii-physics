# ascii-physics


## Running

Building (Updating the regression tests)

> cmake -S . -B build -DUPDATE_GOLDEN=ON
> cmake --build build -j
> cmake --build build --target update_golden

Building (without updating regression tests)

> cmake -S . -B build -DUPDATE_GOLDEN=OFF
> cmake --build build -j
> ctest --test-dir build --output-on-failure 

Building (release)
> cmake -S . -B build-release -DCMAKE_BUILD_TYPE=Release
> cmake --build build-release -j


Configure + build Debug
> cmake --preset debug
> cmake --build --preset debug -j

Configure + build Release
> cmake --preset release
> cmake --build --preset release -j

Configure + build optimized profiling build
> cmake --preset relwithdebinfo
> cmake --build --preset relwithdebinfo -j

Run tests in debug
> ctest --preset debug

Run tests in relwithdebinfo (useful for perf regressions)
> ctest --preset relwithdebinfo


Quick stats
> perf stat ./build-prof/demo_ascii

Profiling
Configure & build (profiler-friendly)
> cmake --preset relwithdebinfo
> cmake --build --preset relwithdebinfo -j

Run under Callgrind (instrumentation off at start)
> cmake --build --preset relwithdebinfo --target profile_demo_ascii

In another terminal, control the window you collect:
pause (ensure it's paused if you started it mid-run)
> callgrind_control -k
warm up the program, then:
> callgrind_control -i      # start collecting
> sleep 1                   # collect ~1s
> callgrind_control -k -d   # stop & dump

Directly call valgrind
> valgrind \
  --tool=callgrind \
  --callgrind-out-file=benchmarks/callgrind.out.%p \
  ./build-prof/demo_ascii
