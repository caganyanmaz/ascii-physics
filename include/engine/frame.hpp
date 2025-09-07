#pragma once

class Frame {
public:
    virtual ~Frame() = default;
    virtual void draw_grid(const char *grid, int grid_height, int grid_widh) = 0;
};