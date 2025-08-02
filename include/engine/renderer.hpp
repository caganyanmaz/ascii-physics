#pragma once
#include <vector>
#include <array>
#include "particle.hpp"
constexpr static int GRID_WIDTH  = 150;
constexpr static int GRID_HEIGHT = 30;

class Renderer {
    char grid[GRID_HEIGHT][GRID_WIDTH + 1];
public:
    void render(const std::vector<Particle>& particles);
    Renderer();
private:
    void update_grid(const std::vector<Particle>& particles);
    void render_grid()const;
    void clear_grid(const std::vector<Particle>& particles);
    std::array<int, 2> get_particle_grid_position(const Particle& particle)const;

};

