#include "engine/renderer.hpp"
#include <iostream>

Renderer::Renderer() {
    for (int i = 0; i < GRID_HEIGHT; i++) {
        for (int j = 0; j < GRID_WIDTH; j++) {
            grid[i][j] = ' ';
        }
        grid[i][GRID_WIDTH] = '\0';
    }
}

void Renderer::render(const std::vector<Particle>& particles) {
    update_grid(particles);
    render_grid();
    clear_grid(particles);
}

void Renderer::debug_render(const std::vector<Particle>& particles)const {
    std::cout << "{\n";
    for (const Particle& particle : particles) {
        std::cout << particle << ",\n";
    }
    std::cout << "}\n";
}

void Renderer::update_grid(const std::vector<Particle>& particles) {
    for (const Particle& particle : particles) {
        auto [x, y] = get_particle_grid_position(particle);
        if (is_in_range(x, y)) {
            grid[y][x] = '.';
        }
    }
}

void Renderer::render_grid() const{
    std::cout << "\x1b[H";
    for (int i = 0; i < GRID_HEIGHT; i++) {
        std::cout << grid[i] << "\n";
    }
}

void Renderer::clear_grid(const std::vector<Particle>& particles) {
    for (const Particle& particle : particles) {
        auto [x, y] = get_particle_grid_position(particle);
        if (is_in_range(x, y)) {
            grid[y][x] = ' ';
        }
    }
}

std::array<int, 2> Renderer::get_particle_grid_position(const Particle& particle) const{
    int x = (particle.position.x + 1) * GRID_WIDTH * 0.5;
    int y = (particle.position.y + 1) * GRID_HEIGHT * 0.5;
    return {x, y};
}

bool Renderer::is_in_range(int x, int y)const {
    return 0 <= x && x < GRID_WIDTH && 0 <= y && y < GRID_HEIGHT;
}
