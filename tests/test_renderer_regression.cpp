#include <gtest/gtest.h>
#include "engine/renderer.hpp"
#include "engine/particle.hpp"
#include "golden.hpp"
#include <memory>

// minimal frame to capture output
class CaptureFrame : public Frame {
public:
    void draw_grid(const char* grid, int h, int w) override {
        buf_.assign(grid, grid + (w + 1) * h); // copy whole grid including '\0's
        H = h; W = w;
    }
    std::string as_string() const {
        // rebuild a string with explicit newlines
        std::ostringstream os;
        for (int y = 0; y < H; ++y) {
            os << (buf_.data() + y * (W + 1)) << "\n";
        }
        return os.str();
    }
private:
    int H=0,W=0;
    std::vector<char> buf_;
};

TEST(RendererRegression, single_particle_at_center) {
    auto cap = std::make_unique<CaptureFrame>();
    CaptureFrame* ptr = cap.get();

    const int W=20,H=10,M=5;
    Renderer r(W,H,M, std::move(cap));

    Particle p; p.position = {0.0,0.0}; p.radius=0.0; p.symbol='@';
    r.render(std::vector<Particle>{p});

    std::string got = ptr->as_string();
    golden::verify("tests/golden/render_center.txt", got);
}