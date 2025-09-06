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

Hotspots with call stacks
> perf record -g --call-graph=dwarf -- ./build-prof/demo_ascii
> perf report

(Optional) Flamegraph
> perf script | stackcollapse-perf.pl > out.folded
> flamegraph.pl out.folded > flame.svg

(Optional) Deep dive
> valgrind --tool=callgrind ./build-prof/demo_ascii
> kcachegrind callgrind.out.*
