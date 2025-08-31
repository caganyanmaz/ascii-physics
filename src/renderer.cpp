#include "engine/renderer.hpp"
#include <cassert>
#include <iostream>

constexpr static char SPRING_CHAR = '.';

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

void Renderer::add_spring(int a, int b) {
    springs.push_back({a, b});
}

void Renderer::debug_render(const std::vector<Particle>& particles)const {
    std::cout << "{\n";
    for (const Particle& particle : particles) {
        std::cout << particle << ",\n";
    }
    std::cout << "}\n";
}

void Renderer::update_grid(const std::vector<Particle>& particles) {
    std::vector<ActiveCell> active_cells = get_all_active_cells(particles);
    for (const ActiveCell& active_cell : active_cells) {
        if (is_in_range(active_cell.position)) {
            grid[active_cell.position.y][active_cell.position.x] = active_cell.draw_symbol;
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
    std::vector<ActiveCell> active_cells = get_all_active_cells(particles);
    for (const ActiveCell& active_cell : active_cells) {
        if (is_in_range(active_cell.position)) {
            grid[active_cell.position.y][active_cell.position.x] = ' ';
        }
    }
}

std::vector<Renderer::ActiveCell> Renderer::get_particle_cells(const Particle& particle) const{
    auto [ center_x, center_y ] = convert_to_screen_coordinates(particle.position);
    double scaled_r = particle.radius * GRID_MULTIPLIER;
    double scaled_r_squared = scaled_r * scaled_r;
    std::vector<ActiveCell> res;
    for (int y = center_y - scaled_r - 1; y <= center_y + scaled_r + 1; y++) {
        for (int x = center_x - scaled_r - 1; x <= center_x + scaled_r + 1; x++) {
            if ((y - center_y) * (y - center_y) + (x - center_x) * (x - center_x) <= (int)scaled_r_squared) {
                res.push_back(ActiveCell(Vec2(x, y), particle.symbol));
            }
        }
    }
    return res;
}


std::vector<Renderer::ActiveCell> Renderer::get_spring_cells(const std::array<int, 2>& spring, const std::vector<Particle>& particles)const {
    assert(0 <= spring[0] && spring[0] < particles.size() && 0 <= spring[1] && spring[1] < particles.size());
    auto source = convert_to_screen_coordinates(particles[spring[0]].position);
    auto target = convert_to_screen_coordinates(particles[spring[1]].position);
    return get_line_cells(source, target, SPRING_CHAR);
}

std::vector<Renderer::ActiveCell> Renderer::get_line_cells(Vec2<int> source, Vec2<int> target, char line_char) const {
    if (source.y < target.y)
        std::swap(source, target);
    const int x_step = source.x < target.x ? 1 : -1;
    std::vector<ActiveCell> res;
    Vec2<int> dist = target - source;
    for (Vec2<int> current = source; current != target;) {
        res.push_back(ActiveCell(current, line_char));
        int next_y = current.y - 1 - source.y;
        int next_x = current.x + x_step - source.x;
        // x is first, hitting vertical wall (or hitting corner)
        if (abs(next_y * dist.x) >= abs(next_x * dist.y)) { 
            current.x += x_step;
        } 
        // y is first hitting horizontal wall (or hitting corner)
        if (abs(next_y * dist.x) <= abs(next_x * dist.y)) {
            current.y--;
        }
    }
    res.push_back(ActiveCell(target, line_char));
    return res;
}

std::vector<Renderer::ActiveCell> Renderer::get_all_active_cells(const std::vector<Particle>& particles)const {
    std::vector<ActiveCell> res;
    for (const std::array<int, 2>& spring : springs) {
        auto cells = get_spring_cells(spring, particles);
        res.insert(res.end(), cells.begin(), cells.end());
    }
    for (const Particle& particle : particles) {
        auto cells = get_particle_cells(particle);
        res.insert(res.end(), cells.begin(), cells.end());
    }
    return res;
}

bool Renderer::is_in_range(Vec2<int> coordinate)const {
    return 0 <= coordinate.x && coordinate.x < GRID_WIDTH && 0 <= coordinate.y && coordinate.y < GRID_HEIGHT;
}

Vec2<int> Renderer::convert_to_screen_coordinates(const Vec2<double>& position)const {
    int x = (position.x * GRID_MULTIPLIER) + GRID_WIDTH * 0.5;
    int y = (position.y * GRID_MULTIPLIER) + GRID_HEIGHT * 0.5;
    return Vec2<int>(x, y);
}