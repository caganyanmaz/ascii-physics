# ascii-physics


## Building

Building (Updating the regression tests)

> cmake -S . -B build -DUPDATE_GOLDEN=ON
> cmake --build build -j
> cmake --build build --target update_golden

Building (without updating regression tests)

> cmake -S . -B build -DUPDATE_GOLDEN=OFF
> cmake --build build -j
> ctest --test-dir build --output-on-failure 