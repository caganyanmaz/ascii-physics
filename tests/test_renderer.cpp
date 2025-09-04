// tests/renderer_frame_test.cpp
#include <gtest/gtest.h>
#include <memory>
#include <vector>
#include "engine/renderer.hpp"       // adjust include path if needed
#include "engine/particle.hpp"
#include "test_utils.hpp"


using namespace test_utils;

// ---------- minimal test frame capturing grids ----------
class TestFrame : public Frame {
public:
    void draw_grid(const char* grid, int grid_height, int grid_width) override {
        // copy (grid_width+1) * grid_height chars, row by row
        h_ = grid_height;
        w_ = grid_width;
        buf_.assign(grid, grid + (grid_width + 1) * grid_height);
        calls_++;
    }

    int width()  const { return w_; }
    int height() const { return h_; }
    int calls()  const { return calls_; }

    // read a cell (x,y). NOTE: rows are (w+1)-strided; row terminator is at index w.
    char at(int x, int y) const {
        return buf_[y * (w_ + 1) + x];
    }
    // the nul terminator of row y
    char terminator(int y) const {
        return buf_[y * (w_ + 1) + w_];
    }
    const std::vector<char>& raw() const { return buf_; }

private:
    int w_ = 0, h_ = 0, calls_ = 0;
    std::vector<char> buf_;
};

// convenience: center coordinates
static int cx(int w) { return w / 2; }
static int cy(int h) { return h / 2; }

// ============ tests ============

TEST(RendererFrameTest, writes_rows_with_nul_terminators_and_dimensions) {
    auto tf = std::make_unique<TestFrame>();
    TestFrame* tf_ptr = tf.get();

    const int W = 20, H = 6, M = 10;
    Renderer r(W, H, M, std::move(tf));

    std::vector<Particle> ps;  // empty scene is fine
    r.render(ps);

    ASSERT_EQ(tf_ptr->width(), W);
    ASSERT_EQ(tf_ptr->height(), H);
    ASSERT_EQ(tf_ptr->calls(), 1);

    // every row must end with '\0'
    for (int y = 0; y < H; ++y) {
        EXPECT_EQ(tf_ptr->terminator(y), '\0') << "row " << y << " missing nul terminator";
    }
}

TEST(RendererFrameTest, maps_world_origin_to_center_and_draws_symbol) {
    auto tf = std::make_unique<TestFrame>();
    TestFrame* tf_ptr = tf.get();

    const int W = 21, H = 9, M = 5;   // odd dims → exact center at (W/2,H/2)
    Renderer r(W, H, M, std::move(tf));

    Particle p;
    p.position = Vec2<double>(0.0, 0.0);   // world origin
    p.radius   = 0.0;                      // draw a single cell
    p.symbol   = '@';

    r.render(std::vector<Particle>{p});

    const int x = cx(W), y = cy(H);
    // expect '@' exactly at the center; everything else should not be '@'
    ASSERT_EQ(tf_ptr->at(x, y), '@');
    for (int yy = 0; yy < H; ++yy) {
        for (int xx = 0; xx < W; ++xx) {
            if (xx == x && yy == y) continue;
            ASSERT_NE(tf_ptr->at(xx, yy), '@') << "unexpected draw at (" << xx << "," << yy << ")";
        }
    }
}

TEST(RendererFrameTest, positive_x_right_positive_y_down_scaling_by_multiplier) {
    auto tf = std::make_unique<TestFrame>();
    TestFrame* tf_ptr = tf.get();

    const int W = 20, H = 10, M = 4;
    Renderer r(W, H, M, std::move(tf));

    // choose positions that land exactly on cell centers: (k/M, k/M)
    Particle p;
    p.position = Vec2<double>( 2.0 / M, 1.0 / M); // → +2 in x, +1 in y from center
    p.radius   = 0.0;
    p.symbol   = 'X';

    r.render(std::vector<Particle>{p});

    const int expected_x = cx(W) + 2;
    const int expected_y = cy(H) + 1; // +y is downward
    ASSERT_EQ(tf_ptr->at(expected_x, expected_y), 'X');
}

TEST(RendererFrameTest, negative_coordinates_map_left_and_up) {
    auto tf = std::make_unique<TestFrame>();
    TestFrame* tf_ptr = tf.get();

    const int W = 22, H = 12, M = 2;
    Renderer r(W, H, M, std::move(tf));

    Particle p;
    p.position = Vec2<double>(-3.0 / M, -2.0 / M); // left 3, up 2 from center
    p.radius   = 0.0;
    p.symbol   = '#';

    r.render(std::vector<Particle>{p});

    const int expected_x = cx(W) - 3;
    const int expected_y = cy(H) - 2; // negative y goes up
    ASSERT_EQ(tf_ptr->at(expected_x, expected_y), '#');
}

TEST(RendererFrameTest, springs_draw_a_line_between_particles_somewhere_between_endpoints) {
    auto tf = std::make_unique<TestFrame>();
    TestFrame* tf_ptr = tf.get();

    const int W = 25, H = 9, M = 5;
    Renderer r(W, H, M, std::move(tf));

    Particle a; a.position = Vec2<double>(0.0,       0.0      ); a.radius = 0.0; a.symbol = 'A';
    Particle b; b.position = Vec2<double>( 4.0 / M,  0.0      ); b.radius = 0.0; b.symbol = 'B';
    // screen endpoints
    const int ax = cx(W),          ay = cy(H);
    const int bx = cx(W) + 4,      by = cy(H);

    r.add_spring(/*a=*/0, /*b=*/1);
    r.render(std::vector<Particle>{a, b});

    // assert endpoints are drawn with their symbols
    ASSERT_EQ(tf_ptr->at(ax, ay), 'A');
    ASSERT_EQ(tf_ptr->at(bx, by), 'B');

    // and at least one interior point on the horizontal between them is non-space
    // (we assume background is ' ' — adjust if your background fill differs)
    bool found_line_pixel = false;
    for (int x = ax + 1; x < bx; ++x) {
        if (tf_ptr->at(x, ay) != ' ' && tf_ptr->at(x, ay) != 'A' && tf_ptr->at(x, ay) != 'B') {
            found_line_pixel = true;
            break;
        }
    }
    EXPECT_TRUE(found_line_pixel) << "expected a spring line between endpoints";
}

TEST(RendererFrameTest, determinism_same_input_same_buffer) {
    auto tf1 = std::make_unique<TestFrame>();
    auto tf2 = std::make_unique<TestFrame>();
    TestFrame* p1 = tf1.get();
    TestFrame* p2 = tf2.get();

    const int W = 30, H = 12, M = 3;
    Renderer r1(W, H, M, std::move(tf1));
    Renderer r2(W, H, M, std::move(tf2));

    Particle p; p.position = Vec2<double>(1.0 / M, -2.0 / M); p.radius = 0.0; p.symbol = 'o';
    std::vector<Particle> scene{p};

    r1.render(scene);
    r2.render(scene);

    ASSERT_EQ(p1->raw().size(), p2->raw().size());
    EXPECT_EQ(p1->raw(), p2->raw());
}
