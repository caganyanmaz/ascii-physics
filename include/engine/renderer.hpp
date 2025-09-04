#pragma once
#include <vector>
#include <array>
#include <memory>
#include "particle.hpp"
#include "terminal_frame.hpp"
constexpr static int GRID_MULTIPLIER = 15;
constexpr static int GRID_WIDTH  = 150;
constexpr static int GRID_HEIGHT = 30;


class Renderer {
    struct ActiveCell {
        Vec2<int> position;
        char draw_symbol;
        ActiveCell(Vec2<int> position, char draw_symbol) : position(position), draw_symbol(draw_symbol) {}
    };
    std::vector<char> grid;
    std::vector<std::array<int, 2>> springs;
    std::unique_ptr<Frame> frame;
    const int grid_width, grid_height, grid_multiplier;
public:
    void render(const std::vector<Particle>& particles);
    void debug_render(const std::vector<Particle>& particles)const;
    void add_spring(int a, int b);
    Renderer();
    Renderer(int grid_width, int grid_height, int grid_multiplier, std::unique_ptr<Frame> frame);
private:
    void update_grid(const std::vector<Particle>& particles);
    void render_grid();
    void clear_grid(const std::vector<Particle>& particles);
    bool is_in_range(Vec2<int> coordinate)const;
    std::vector<ActiveCell> get_spring_cells(const std::array<int, 2>& spring, const std::vector<Particle>& particles)const;
    std::vector<ActiveCell> get_particle_cells(const Particle& particle)const;
    std::vector<ActiveCell> get_line_cells(Vec2<int> source, Vec2<int> target, char line_char)const;
    std::vector<ActiveCell> get_all_active_cells(const std::vector<Particle>& particles)const;
    Vec2<int> convert_to_screen_coordinates(const Vec2<double>& position)const;
    char& get_cell(int x, int y);
};

