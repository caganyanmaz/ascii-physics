#include "engine/particle.hpp"

std::ostream& operator<<(std::ostream& os, const Particle& particle) {
    return os << "{position: " << particle.position << ", velocity:" << particle.velocity << "}";
}