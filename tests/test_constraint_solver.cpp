#include <gtest/gtest.h>
#include <memory>
#include <vector>
#include "engine/constraint_solver.hpp"            // ConstraintSolver
#include "engine/particle.hpp"                     // Particle
#include "engine/math.hpp"                         // your Vec2<double> header if separate
#include "engine/detail/constraint_solver.hpp"     // get_block_subspan, bicg, dot/norm
#include "engine/sparse_matrix.hpp"                // SparseMatrix::Block
#include "engine/constraint.hpp"                   // abstract Constraint

static constexpr double TOL = 1e-12;

// -------------------- Simple concrete mock constraints --------------------
// Each constraint contributes 1 scalar row, acting on 1 particle (2 DOF: x,y).
// J row for Y-constraint is [0 1] at the particle’s column; J̇ = 0.
// C = y, Ċ = vy.
// J row for X-constraint is [1 0]; C = x, Ċ = vx.
// Blocks are 1x2 located at row = its constraint row (solver assigns in its block list)
// and col = 2*particle_index.

class MockYConstraint : public Constraint {
public:
    MockYConstraint(int particle_index, int constraint_row_i)
        : pidx(particle_index), row_i(constraint_row_i) {}

    std::vector<SparseMatrix::Block> create_j_blocks() override {
        // 1×2 block at (row_i, 2*pidx)
        return { SparseMatrix::Block{ row_i, 2*pidx, 1, 2, std::vector<double>(2, 0.0) } };
    }
    std::vector<SparseMatrix::Block> create_j_dot_blocks() override {
        return { SparseMatrix::Block{ row_i, 2*pidx, 1, 2, std::vector<double>(2, 0.0) } };
    }
    void update_j_blocks(const std::vector<Particle>&, std::span<SparseMatrix::Block> jb) override {
        ASSERT_EQ(jb.size(), 1u);
        // J row = [0 1]
        jb[0].data[0] = 0.0;
        jb[0].data[1] = 1.0;
    }
    void update_j_dot_blocks(const std::vector<Particle>&, std::span<SparseMatrix::Block> jdb) override {
        ASSERT_EQ(jdb.size(), 1u);
        // J̇ = 0
        jdb[0].data[0] = 0.0;
        jdb[0].data[1] = 0.0;
    }
    double return_c(const std::vector<Particle>& ps) override { return ps[pidx].position.y; }
    double return_c_dot(const std::vector<Particle>& ps) override { return ps[pidx].velocity.y; }
private:
    int pidx;
    int row_i;
};

class MockXConstraint : public Constraint {
public:
    MockXConstraint(int particle_index, int constraint_row_i)
        : pidx(particle_index), row_i(constraint_row_i) {}

    std::vector<SparseMatrix::Block> create_j_blocks() override {
        return { SparseMatrix::Block{ row_i, 2*pidx, 1, 2, std::vector<double>(2, 0.0) } };
    }
    std::vector<SparseMatrix::Block> create_j_dot_blocks() override {
        return { SparseMatrix::Block{ row_i, 2*pidx, 1, 2, std::vector<double>(2, 0.0) } };
    }
    void update_j_blocks(const std::vector<Particle>&, std::span<SparseMatrix::Block> jb) override {
        ASSERT_EQ(jb.size(), 1u);
        // J row = [1 0]
        jb[0].data[0] = 1.0;
        jb[0].data[1] = 0.0;
    }
    void update_j_dot_blocks(const std::vector<Particle>&, std::span<SparseMatrix::Block> jdb) override {
        ASSERT_EQ(jdb.size(), 1u);
        // J̇ = 0
        jdb[0].data[0] = 0.0;
        jdb[0].data[1] = 0.0;
    }
    double return_c(const std::vector<Particle>& ps) override { return ps[pidx].position.x; }
    double return_c_dot(const std::vector<Particle>& ps) override { return ps[pidx].velocity.x; }
private:
    int pidx;
    int row_i;
};


// -------------------- ConstraintSolver::solve — single constraint (Y) --------------------

TEST(ConstraintSolver, Solve_SingleYConstraint_AddsExpectedForce) {
    // One particle, mass = 2.0 (applies to both x,y masses in diagonal M).
    std::vector<Particle> particles(1);
    particles[0].mass = 2.0;
    particles[0].position = { 0.25,  -0.5 };  // x, y
    particles[0].velocity = { 1.0,   -0.25 }; // vx, vy
    particles[0].force_accumulator = { 3.0,  7.0 }; // Q = (Fx, Fy)

    // One Y constraint on particle 0. Spring/damping for drift correction:
    const double ks = 10.0;
    const double kd = 4.0;

    std::vector<std::unique_ptr<Constraint>> constraints;
    constraints.push_back(std::make_unique<MockYConstraint>(0, 0));

    ConstraintSolver solver(std::move(constraints), /*particle_count=*/1, /*spring_constant=*/ks, /*damping_constant=*/kd);
    solver.solve(particles);

    // Expected from JWJ^T = (1/m) for Y-constraint with J=[0 1],
    // RHS = -J̇ q̇ - JWQ - ks*C - kd*Ċ = -(0) - (Fy/m) - ks*y - kd*vy
    // m = 2, Fy = 7, y = -0.5, vy = -0.25
    // RHS = - (7/2) - 10*(-0.5) - 4*(-0.25) = -3.5 + 5 + 1 = 2.5
    // (1/m) * λ = RHS  =>  (1/2) λ = 2.5  =>  λ = 5.0
    // Q̂ = J^T λ = (0, λ) = (0, 5). New force = (3, 7) + (0, 5) = (3, 12).
    EXPECT_NEAR(particles[0].force_accumulator.x, 3.0, TOL);
    EXPECT_NEAR(particles[0].force_accumulator.y, 12.0, TOL);
}

// -------------------- ConstraintSolver::solve — two independent constraints (X & Y) -----

TEST(ConstraintSolver, Solve_TwoConstraints_XandY_IndependentLambdas) {
    std::vector<Particle> particles(1);
    particles[0].mass = 1.5;
    particles[0].position = { -0.2,  0.3 };
    particles[0].velocity = {  0.4, -0.1 };
    particles[0].force_accumulator = { -2.0, 5.0 }; // Q

    const double ks = 8.0;
    const double kd = 1.5;

    std::vector<std::unique_ptr<Constraint>> constraints;
    constraints.push_back(std::make_unique<MockXConstraint>(0, 0)); // Cx row 0
    constraints.push_back(std::make_unique<MockYConstraint>(0,1)); // Cy row 1


    ConstraintSolver solver(std::move(constraints), 1, ks, kd);
    solver.solve(particles);

    // For both constraints J=[1 0] and J=[0 1]; JWJ^T = (1/m) each, decoupled.
    // m = 1.5.
    // λx: RHS = -(Fx/m) - ks*x - kd*vx
    //     = -(-2/1.5) - 8*(-0.2) - 1.5*(0.4)
    //     = ( 1.3333333333333333 ) + 1.6 - 0.6 = 2.3333333333333335
    // (1/m)λx = RHS -> (2/3) λx = 2.3333333333333335 -> λx = 3.5
    //
    // λy: RHS = -(Fy/m) - ks*y - kd*vy
    //     = -( 5/1.5 ) - 8*(0.3) - 1.5*(-0.1)
    //     = -3.3333333333333335 - 2.4 + 0.15 = -5.583333333333333
    // (2/3) λy = -5.583333333333333 -> λy = -8.375
    //
    // Q̂ = (λx, λy) = (3.5, -8.375)
    // New force = (-2, 5) + (3.5, -8.375) = (1.5, -3.375)
    EXPECT_NEAR(particles[0].force_accumulator.x,  1.5, TOL);
    EXPECT_NEAR(particles[0].force_accumulator.y, -3.375, TOL);
}

// -------------------- get_block_subspan (layout utility) --------------------

TEST(ConstraintDetail, GetBlockSubspan_ReturnsCorrectSegments) {
    // blocks: [ c0 has 2 blocks ][ c1 has 1 block ][ c2 has 3 blocks ]
    std::vector<SparseMatrix::Block> blocks;
    blocks.push_back(SparseMatrix::Block{0,0,1,2,{1,2}});
    blocks.push_back(SparseMatrix::Block{1,2,1,2,{3,4}});
    blocks.push_back(SparseMatrix::Block{2,0,1,2,{5,6}});
    blocks.push_back(SparseMatrix::Block{3,0,1,2,{7,8}});
    blocks.push_back(SparseMatrix::Block{4,0,1,2,{9,10}});
    blocks.push_back(SparseMatrix::Block{5,0,1,2,{11,12}});

    std::vector<int> starts = {0, 2, 3, 6}; // c0 starts at 0 (len 2), c1 at 2 (len 1), c2 at 3 (len 3)

    // c0
    {
        std::span<SparseMatrix::Block> all(blocks.data(), blocks.size());
        std::span<SparseMatrix::Block> sub = get_block_subspan(all, starts, /*constraint_index=*/0);
        ASSERT_EQ(sub.size(), 2u);
        EXPECT_EQ(sub[0].data[0], 1.0);
        EXPECT_EQ(sub[1].data[0], 3.0);
    }
    // c1
    {
        std::span<SparseMatrix::Block> all(blocks.data(), blocks.size());
        std::span<SparseMatrix::Block> sub = get_block_subspan(all, starts, 1);
        ASSERT_EQ(sub.size(), 1u);
        EXPECT_EQ(sub[0].data[0], 5.0);
    }
    // c2
    {
        std::span<SparseMatrix::Block> all(blocks.data(), blocks.size());
        std::span<SparseMatrix::Block> sub = get_block_subspan(all, starts, 2);
        ASSERT_EQ(sub.size(), 3u);
        EXPECT_EQ(sub[0].data[0], 7.0);
        EXPECT_EQ(sub[1].data[0], 9.0);
        EXPECT_EQ(sub[2].data[0], 11.0);
    }
}

// -------------------- vector_dot / vector_norm_squared --------------------

TEST(ConstraintDetail, VectorDot_Basic) {
    std::vector<double> a = {1.0, -2.0, 3.0};
    std::vector<double> b = {4.0,  0.5, -1.0};
    // 1*4 + (-2)*0.5 + 3*(-1) = 4 -1 -3 = 0
    double d = vector_dot(a, b);
    EXPECT_NEAR(d, 0.0, TOL);
}

#if GTEST_HAS_DEATH_TEST
TEST(ConstraintDetailDeath, VectorDot_SizeMismatchAsserts) {
    std::vector<double> a = {1.0, 2.0};
    std::vector<double> b = {3.0};
    EXPECT_DEATH(
        {
            volatile double d = vector_dot(a, b);
            (void)d;
        },
        ".*"
    );
}
#endif

TEST(ConstraintDetail, VectorNormSquared_Basic) {
    std::vector<double> v = {2.0, -3.0, 6.0};
    // 4 + 9 + 36 = 49
    double n2 = vector_norm_squared(v);
    EXPECT_NEAR(n2, 49.0, TOL);
}

// -------------------- BiCG for symmetric matrix (Ax=b) --------------------

// ------------------ SPD case: 2x2, solves exactly ------------------
// A = [[4,1],[1,3]] (SPD), b = [1,2]
// det = 11; A^{-1} = (1/11)*[[3,-1],[-1,4]]
// x* = (1/11)*[1,7] = [0.090909..., 0.636363...]
TEST(BiCGSym, SPD_2x2_KnownSolution) {
    auto Ax = [](const std::vector<double>& x) {
        std::vector<double> y(2);
        y[0] = 4.0 * x[0] + 1.0 * x[1];
        y[1] = 1.0 * x[0] + 3.0 * x[1];
        return y;
    };
    std::vector<double> b = {1.0, 2.0};

    std::vector<double> x = biconjugate_gradient_method_for_symmetric_matrix(Ax, b);

    ASSERT_EQ(x.size(), 2u);
    EXPECT_NEAR(x[0], 1.0/11.0, TOL);
    EXPECT_NEAR(x[1], 7.0/11.0, TOL);
}

// ------------------ PSD case: 3x3 diagonal with a zero eigenvalue ------------------
// A = diag(2, 0, 5) (symmetric PSD).
// Choose b in range(A): b = [ 4, 0, 15 ].
// The system Ax = b has the (component-wise) equations:
//  2*x0 =  4  -> x0 = 2
//  0*x1 =  0  -> any x1 is ok; the minimum-norm solution has x1 = 0
//  5*x2 = 15  -> x2 = 3
// We check the natural/min-norm solution [2, 0, 3].
TEST(BiCGSym, PSD_Diagonal_Consistent_MinNormSolution) {
    // capture diagonal entries in the lambda
    const double d0 = 2.0, d1 = 0.0, d2 = 5.0;
    auto Ax = [=](const std::vector<double>& x) {
        std::vector<double> y(3);
        y[0] = d0 * x[0];
        y[1] = d1 * x[1];
        y[2] = d2 * x[2];
        return y;
    };
    std::vector<double> b = {4.0, 0.0, 15.0};

    std::vector<double> x = biconjugate_gradient_method_for_symmetric_matrix(Ax, b);

    ASSERT_EQ(x.size(), 3u);
    // Expect the obvious solution; CG-type methods typically converge to the minimum-norm one in the nullspace.
    EXPECT_NEAR(x[0], 2.0, TOL);
    EXPECT_NEAR(x[1], 0.0, TOL);
    EXPECT_NEAR(x[2], 3.0, TOL);

    // Quick residual check: A*x ≈ b
    std::vector<double> r = Ax(x);
    EXPECT_NEAR(r[0], b[0], 1e-10);
    EXPECT_NEAR(r[1], b[1], 1e-10);
    EXPECT_NEAR(r[2], b[2], 1e-10);
}

// ------------------ Larger SPD: 3x3, hand-computed inverse result ------------------
// A = [[6,2,0],
//      [2,5,1],
//      [0,1,4]]  (symmetric, strictly diagonally dominant -> SPD)
// b = [ 8, 3, 7 ]
// Solve by hand (or check numerically once and freeze the expected literals):
// Using elimination one finds x* = [ 1, 0, 1.5 ] exactly.
// (You can verify: A*[1,0,1.5] = [6*1+2*0+0*1.5, 2*1+5*0+1*1.5, 0*1+1*0+4*1.5] = [8, 3.5, 6]
// To make b exactly match, pick b = [8, 3.5, 6].)
TEST(BiCGSym, SPD_3x3_EasyStructure) {
    auto Ax = [](const std::vector<double>& x) {
        std::vector<double> y(3);
        y[0] = 6.0*x[0] + 2.0*x[1] + 0.0*x[2];
        y[1] = 2.0*x[0] + 5.0*x[1] + 1.0*x[2];
        y[2] = 0.0*x[0] + 1.0*x[1] + 4.0*x[2];
        return y;
    };
    // Choose b to make an exact integer solution x* = [1, 0, 1.5]
    std::vector<double> b = {6.0, 3.5, 6.0};

    std::vector<double> x = biconjugate_gradient_method_for_symmetric_matrix(Ax, b);

    ASSERT_EQ(x.size(), 3u);
    EXPECT_NEAR(x[0], 1.0, TOL);
    EXPECT_NEAR(x[1], 0.0, TOL);
    EXPECT_NEAR(x[2], 1.5, TOL);
}