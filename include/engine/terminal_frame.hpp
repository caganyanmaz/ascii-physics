#pragma once
#include "frame.hpp"

class TerminalFrame : public Frame {
public:
    void draw_grid(const char *grid, int grid_height, int grid_widh)override;
};