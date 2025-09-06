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


static Particle make_particle(Vec2<double> pos, char symbol, double radius = 0.0) {
    Particle p;
    p.position = pos;
    p.velocity = Vec2<double>(0.0, 0.0);
    p.radius   = radius;
    p.symbol   = symbol;
    return p;
}

TEST(RendererRegression, single_particle_at_center) {
    auto cap = std::make_unique<CaptureFrame>();
    CaptureFrame* ptr = cap.get();

    const int W=20,H=10,M=5;
    Renderer r(W, H, M, std::move(cap));

    Particle p; p.position = {0.0,0.0}; p.radius=0.0; p.symbol='@';
    r.render(std::vector<Particle>{p});

    std::string got = ptr->as_string();
    golden::verify("render/render_center.txt", got);
}

// 1) center, 4 corners (within bounds), and one OOB: stable composition check
TEST(RendererRegression, mosaic_corners_and_center) {
    auto cap = std::make_unique<CaptureFrame>();
    auto* ptr = cap.get();
    const int W=31, H=13, M=5;
    Renderer r(W, H, M, std::move(cap));

    // choose exact-cell coords: (k/M)
    const int cx = W/2, cy = H/2;
    std::vector<Particle> scene = {
        make_particle({ 0.0,            0.0           }, '@'), // center
        make_particle({-double(cx)/M,  -double(cy)/M }, 'L'), // top-left
        make_particle({ double(cx-1)/M,-double(cy)/M }, 'R'), // top-right (in-bounds at x=W-2)
        make_particle({-double(cx)/M,   double(cy-1)/M}, 'T'),// bottom-left
        make_particle({ double(cx-1)/M, double(cy-1)/M}, 'B'),// bottom-right
        make_particle({ double(cx+10)/M, 0.0 }, 'X')          // OOB to the right
    };

    r.render(scene);
    golden::verify("render/mosaic_corners_and_center.txt", ptr->as_string());
}

// 2) horizontal spring: ensure a visible line between A and B with endpoints intact
TEST(RendererRegression, spring_horizontal_midline) {
    auto cap = std::make_unique<CaptureFrame>();
    auto* ptr = cap.get();
    const int W=40, H=9, M=4;
    Renderer r(W, H, M, std::move(cap));

    Particle a = make_particle({-6.0/M, 0.0}, 'A');
    Particle b = make_particle({ 6.0/M, 0.0}, 'B');

    r.add_spring(0, 1);
    r.render({a, b});

    golden::verify("render/spring_horizontal_midline.txt", ptr->as_string());
}

// 3) diagonal spring: sanity over line rasterization (no re-implementing math, just snapshot)
TEST(RendererRegression, spring_diagonal) {
    auto cap = std::make_unique<CaptureFrame>();
    auto* ptr = cap.get();
    const int W=41, H=17, M=5;
    Renderer r(W, H, M, std::move(cap));

    Particle a = make_particle({-6.0/M, -4.0/M}, 'a');
    Particle b = make_particle({ 7.0/M,  5.0/M}, 'b');

    r.add_spring(0, 1);
    r.render({a, b});

    golden::verify("render/spring_diagonal.txt", ptr->as_string());
}

// 4) overlapping particles: last-wins (or whatever your impl does) — snapshot the result
TEST(RendererRegression, overlapping_order) {
    auto cap = std::make_unique<CaptureFrame>();
    auto* ptr = cap.get();
    const int W=21, H=11, M=3;
    Renderer r(W, H, M, std::move(cap));

    // all at center; draw order = vector order
    Particle p1 = make_particle({0.0, 0.0}, '1');
    Particle p2 = make_particle({0.0, 0.0}, '2');
    Particle p3 = make_particle({0.0, 0.0}, '3');

    r.render({p1, p2, p3});
    golden::verify("render/overlapping_order.txt", ptr->as_string());
}

// 5) simple disc (radius=1) — freeze whatever raster rule you currently use
TEST(RendererRegression, tiny_disc_radius_one) {
    auto cap = std::make_unique<CaptureFrame>();
    auto* ptr = cap.get();
    const int W=25, H=13, M=5;
    Renderer r(W, H, M, std::move(cap));

    Particle p = make_particle({0.0, 0.0}, 'o', /*radius=*/1.0);
    r.render({p});

    golden::verify("render/tiny_disc_radius_one.txt", ptr->as_string());
}
