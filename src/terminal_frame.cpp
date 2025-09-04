#include "engine/terminal_frame.hpp"
#include <stdio.h>
#include <iostream>

void TerminalFrame::draw_grid(const char *grid, int grid_height, int grid_width) {
    std::cout << "\x1b[H";
    for (int i = 0; i < grid_height; i++) {
        std::cout << grid << "\n";
        grid += grid_width + 1;
    }
}