#include "engine/surface.hpp"

std::ostream& operator<<(std::ostream& os, const Surface& surface) {
    return os << "{position: " << surface.position << ", normal: " << surface.normal << "}";
}