#include "engine/particle.hpp"
#include <cmath>
#include <algorithm>

constexpr double TOL = 1e-9;
constexpr int TOTALLY_CONSTRAINED = 3;
constexpr int COLLINEAR_CONSTRAINTS = 4;

// We can add multiple normal constraints (this means there's a normal force from the normal direction)
// Depending on the number of normal constraints added, there are multiple possibilities:
// No normal constraint
// One normal constraint
// Two normal constraints that are not collinear (where they also restrain forces the range between them)
// Two normal constraints that are collinear + another normal constraint that is in the range
// Two normal constraints that are collinear (they only restrain forces in their direction)
// Totally restricted (three forces that surround the particle)
// Note that just because there are more than two normal surfaces acting on the particle, it doesn't mean that 
// the forces will totally constrain the object
#include <iostream>
void Particle::add_normal(Vec2<double> normal) {
    assert(std::abs(normal.norm() - 1) < TOL);
    if (normal_count == TOTALLY_CONSTRAINED)
        return;
    if (normal_count < 2) {
        normals[normal_count++] = normal;
        if (normal_count == 2 && std::abs(normals[0] * normals[1] + 1) < TOL) {
            normal_count = COLLINEAR_CONSTRAINTS;
        }
        return;
    }
    std::array<Vec2<double>, 3> current_normals({
        normals[0],
        normals[1],
        normals[2]
    });
    std::array<std::pair<double, int>, 3> normal_angles{{
        { current_normals[0].angle(), 0 },
        { current_normals[1].angle(), 1 },
        { current_normals[2].angle(), 2 }
    }};
    std::sort(normal_angles.begin(), normal_angles.end());
    std::array<double, 3> gaps;
    gaps[0] = normal_angles[2].first - normal_angles[1].first;
    gaps[1] = normal_angles[0].first + 2 * M_PI - normal_angles[2].first;
    gaps[2] = normal_angles[1].first - normal_angles[0].first;
    int max_gap_index = std::distance(gaps.begin(), std::max_element(gaps.begin(), gaps.end()));
    if (gaps[max_gap_index] > M_PI) {
        normal_count = TOTALLY_CONSTRAINED;
        return;
    }
    normal_count = 0;
    for (int i = 0; i < 3; i++) {
        if (i == max_gap_index)
            continue;
        normals[normal_count++] = current_normals[normal_angles[i].second];
    }
    Vec2 excluded = current_normals[normal_angles[max_gap_index].second];
    if (std::abs(normals[0] * normals[1] + 1) < TOL && (std::abs(normals[0] * excluded) < TOL || std::abs(normals[1] * excluded) < TOL)) {
        normal_count = COLLINEAR_CONSTRAINTS;
    }
}

void Particle::flush_normals() {
    normal_count = 0;
}

Particle::NormalState Particle::get_normal_state()const {
    if (normal_count <= 2)
        return NormalState::REGULAR;
    if (normal_count == TOTALLY_CONSTRAINED)
        return NormalState::CONSTRAINED;
    if (normal_count == COLLINEAR_CONSTRAINTS)
        return NormalState::COLLINEAR;
    assert(false);
}

std::ostream& operator<<(std::ostream& os, const Particle& particle) {
    return os << "{position: " << particle.position << ", velocity:" << particle.velocity << "}";
}
    
