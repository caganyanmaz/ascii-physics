#include "math.hpp"
#include <ostream>
// Creates an infinite line, and every object must be at one side of the line 
// (Used for creating boundaries, and potentially separating stuff in the future)

struct Surface {
    Vec2<double> position;
    Vec2<double> normal;
    Surface(Vec2<double> position, Vec2<double> normal) : position(position), normal(normal) {}
};

std::ostream& operator<<(std::ostream& os, const Surface& surface);