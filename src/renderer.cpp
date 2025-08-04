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
        auto positions = get_particle_grid_positions(particle);
        for (auto [x, y] : positions) {
            if (is_in_range(x, y)) {
                grid[y][x] = particle.symbol;
            }
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
        auto positions = get_particle_grid_positions(particle);
        for (auto [x, y] : positions) {
            if (is_in_range(x, y)) {
                grid[y][x] = ' ';
            }
        }
    }
}

std::vector<std::array<int, 2>> Renderer::get_particle_grid_positions(const Particle& particle) const{
    int center_x = (particle.position.x * GRID_MULTIPLIER) + GRID_WIDTH * 0.5;
    int center_y = (particle.position.y * GRID_MULTIPLIER) + GRID_HEIGHT * 0.5;
    double scaled_r = particle.radius * GRID_MULTIPLIER;
    double scaled_r_squared = scaled_r * scaled_r;
    std::vector<std::array<int, 2>> res;
    for (int y = center_y - scaled_r - 1; y <= center_y + scaled_r + 1; y++) {
        for (int x = center_x - scaled_r - 1; x <= center_x + scaled_r + 1; x++) {
            if ((y - center_y) * (y - center_y) + (x - center_x) * (x - center_x) <= (int)scaled_r_squared) {
                res.push_back({x, y});
            }
        }
    }
    return res;
}

bool Renderer::is_in_range(int x, int y)const {
    return 0 <= x && x < GRID_WIDTH && 0 <= y && y < GRID_HEIGHT;
}
