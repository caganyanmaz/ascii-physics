#include <gtest/gtest.h>
#include <vector>
#include "engine/sparse_matrix.hpp"
#include "engine/detail/sparse_matrix.hpp"

static constexpr double TOL = 1e-12;

// ---------------------- SparseMatrix::right_multiply_with_vector ----------------------

TEST(SparseMatrix, RightMultiply_SingleBlock_Center) {
    SparseMatrix A(4, 5);

    SparseMatrix::Block b;
    b.i = 1; b.j = 1; b.ilength = 2; b.jlength = 3;
    b.data = {
        1.0, 2.0, 3.0,
        4.0, 5.0, 6.0
    }; // row-major
    A.blocks.push_back(b);

    std::vector<double> x = {10, 20, 30, 40, 50}; // n=5
    std::vector<double> y = A.right_multiply_with_vector(x);

    // Expected:
    // row 1: [1 2 3]·[20 30 40] = 1*20 + 2*30 + 3*40 = 200
    // row 2: [4 5 6]·[20 30 40] = 4*20 + 5*30 + 6*40 = 470
    ASSERT_EQ(y.size(), 4u);
    EXPECT_NEAR(y[0], 0.0, TOL);
    EXPECT_NEAR(y[1], 200.0, TOL);
    EXPECT_NEAR(y[2], 470.0, TOL);
    EXPECT_NEAR(y[3], 0.0, TOL);
}

TEST(SparseMatrix, RightMultiply_MultipleBlocks_Disjoint) {
    SparseMatrix A(5, 6);

    A.blocks.push_back(SparseMatrix::Block{
        0, 0, 2, 2,
        { 1.0, 2.0,
          3.0, 4.0 }
    });

    A.blocks.push_back(SparseMatrix::Block{
        2, 3, 3, 2,
        { 5.0,  6.0,
          7.0,  8.0,
          9.0, 10.0 }
    });

    std::vector<double> x = {1, 2, 3, 4, 5, 6}; // n=6
    std::vector<double> y = A.right_multiply_with_vector(x);

    // Expected:
    // row0: [1 2]·[1 2] = 1 + 4 = 5
    // row1: [3 4]·[1 2] = 3 + 8 = 11
    // row2: [5 6]·[4 5] = 20 + 30 = 50
    // row3: [7 8]·[4 5] = 28 + 40 = 68
    // row4: [9 10]·[4 5] = 36 + 50 = 86
    ASSERT_EQ(y.size(), 5u);
    EXPECT_NEAR(y[0],  5.0, TOL);
    EXPECT_NEAR(y[1], 11.0, TOL);
    EXPECT_NEAR(y[2], 50.0, TOL);
    EXPECT_NEAR(y[3], 68.0, TOL);
    EXPECT_NEAR(y[4], 86.0, TOL);
}

TEST(SparseMatrix, RightMultiply_BlockTouchingEdges) {
    SparseMatrix A(3, 4);

    A.blocks.push_back(SparseMatrix::Block{
        1, 1, 2, 3,
        { 1.0, 0.0, 2.0,
          0.0, 3.0, 4.0 }
    });

    std::vector<double> x = {0.5, -1.0, 2.0, 3.0};
    std::vector<double> y = A.right_multiply_with_vector(x);

    // Expected:
    // row1: [1 0 2]·[-1 2 3] = -1 + 0 + 6 = 5
    // row2: [0 3 4]·[-1 2 3] = 0 + 6 + 12 = 18
    ASSERT_EQ(y.size(), 3u);
    EXPECT_NEAR(y[0], 0.0, TOL);
    EXPECT_NEAR(y[1], 5.0, TOL);
    EXPECT_NEAR(y[2], 18.0, TOL);
}

// ------------------- SparseMatrix::right_multiply_transpose_with_vector ----------------

TEST(SparseMatrix, TransposeMultiply_SingleBlock) {
    SparseMatrix A(4, 5);

    A.blocks.push_back(SparseMatrix::Block{
        1, 2, 3, 2,
        { 2.0, 1.0,
          0.0, 3.0,
          4.0, 5.0 }
    });

    std::vector<double> x = {7.0, -2.0, 0.5, 1.0}; // m=4
    std::vector<double> z = A.right_multiply_transpose_with_vector(x);

    // Only columns 2 and 3 affected:
    // z[2] = 2*x[1] + 0*x[2] + 4*x[3] = 2*(-2) + 0 + 4*1 = 0
    // z[3] = 1*x[1] + 3*x[2] + 5*x[3] = -2 + 1.5 + 5 = 4.5
    ASSERT_EQ(z.size(), 5u);
    EXPECT_NEAR(z[0], 0.0, TOL);
    EXPECT_NEAR(z[1], 0.0, TOL);
    EXPECT_NEAR(z[2], 0.0, TOL);
    EXPECT_NEAR(z[3], 4.5, TOL);
    EXPECT_NEAR(z[4], 0.0, TOL);
}

// --------------------------- Overlap probe (sum behavior) ---------------------------

TEST(SparseMatrix, OverlappingBlocks_SumProbe) {
    SparseMatrix A(4, 4);

    A.blocks.push_back(SparseMatrix::Block{
        1, 1, 2, 2,
        { 1.0, 2.0,
          3.0, 4.0 }
    });
    A.blocks.push_back(SparseMatrix::Block{
        2, 2, 2, 2,
        { 10.0, 20.0,
          30.0, 40.0 }
    });

    std::vector<double> x = {1.0, 2.0, 3.0, 4.0}; // n=4
    std::vector<double> y = A.right_multiply_with_vector(x);

    // Expected (sum overlaps):
    // Block1 contributions:
    //  row1 += [1 2]·[2 3] = 1*2 + 2*3 = 8
    //  row2 += [3 4]·[2 3] = 3*2 + 4*3 = 18
    // Block2 contributions:
    //  row2 += [10 20]·[3 4] = 10*3 + 20*4 = 110  => row2 total 128
    //  row3 += [30 40]·[3 4] = 30*3 + 40*4 = 250
    ASSERT_EQ(y.size(), 4u);
    EXPECT_NEAR(y[0],   0.0, TOL);
    EXPECT_NEAR(y[1],   8.0, TOL);
    EXPECT_NEAR(y[2], 128.0, TOL);
    EXPECT_NEAR(y[3], 250.0, TOL);
}

// ------------------------------- Zero blocks behavior -------------------------------

TEST(SparseMatrix, NoBlocks_ZeroMatrix) {
    SparseMatrix A(3, 5);

    std::vector<double> x = {1, 2, 3, 4, 5};
    std::vector<double> y = A.right_multiply_with_vector(x);
    ASSERT_EQ(y.size(), 3u);
    EXPECT_NEAR(y[0], 0.0, TOL);
    EXPECT_NEAR(y[1], 0.0, TOL);
    EXPECT_NEAR(y[2], 0.0, TOL);

    std::vector<double> v = {-1.0, 0.5, 2.0};
    std::vector<double> z = A.right_multiply_transpose_with_vector(v);
    ASSERT_EQ(z.size(), 5u);
    EXPECT_NEAR(z[0], 0.0, TOL);
    EXPECT_NEAR(z[1], 0.0, TOL);
    EXPECT_NEAR(z[2], 0.0, TOL);
    EXPECT_NEAR(z[3], 0.0, TOL);
    EXPECT_NEAR(z[4], 0.0, TOL);
}

// ------------------------------------ Death tests ------------------------------------

#if GTEST_HAS_DEATH_TEST
TEST(SparseMatrixDeathTest, RightMultiply_SizeMismatch_Aborts) {
    SparseMatrix A(3, 4);
    A.blocks.push_back(SparseMatrix::Block{0,0,1,1, {1.0}});

    std::vector<double> x_bad = {1.0, 2.0, 3.0}; // should be size 4
    EXPECT_DEATH(
        {
            volatile auto tmp = A.right_multiply_with_vector(x_bad);
            (void)tmp;
        },
        ".*"
    );
}

TEST(SparseMatrixDeathTest, TransposeMultiply_SizeMismatch_Aborts) {
    SparseMatrix A(3, 4);
    A.blocks.push_back(SparseMatrix::Block{0,0,1,1, {1.0}});

    std::vector<double> x_bad = {1.0, 2.0, 3.0, 4.0}; // should be size 3
    EXPECT_DEATH(
        {
            volatile auto tmp = A.right_multiply_transpose_with_vector(x_bad);
            (void)tmp;
        },
        ".*"
    );
}
#endif  // GTEST_HAS_DEATH_TEST

// ===================== detail helper tests (no helpers, all hard-coded) =====================

// add_block_multiplication_to_result: y += B*x (single block)
TEST(SparseMatrixDetail, AddBlockMul_BasicCenter) {
    const SparseMatrix::Block b = {
        1, 2, 2, 3,
        { 1.0, 2.0, 3.0,
          4.0, 5.0, 6.0 }
    };
    std::vector<double> x = {1, 2, 3, 4, 5, 6, 7}; // n=7
    std::vector<double> y(5, 0.0);

    add_block_multiplication_to_result(b, x, y);

    // row1: [1 2 3]·[3 4 5] = 3 + 8 + 15 = 26
    // row2: [4 5 6]·[3 4 5] = 12 + 20 + 30 = 62
    ASSERT_EQ(y.size(), 5u);
    EXPECT_NEAR(y[0],  0.0, TOL);
    EXPECT_NEAR(y[1], 26.0, TOL);
    EXPECT_NEAR(y[2], 62.0, TOL);
    EXPECT_NEAR(y[3],  0.0, TOL);
    EXPECT_NEAR(y[4],  0.0, TOL);
}

TEST(SparseMatrixDetail, AddBlockMul_AccumulatesIntoExistingResult) {
    const SparseMatrix::Block b1 = { 0, 2, 1, 3, {1.0, 2.0, 3.0} };
    const SparseMatrix::Block b2 = { 3, 2, 1, 3, {4.0, 5.0, 6.0} };
    std::vector<double> x = {0, 0, 1, 2, 3, 0};
    std::vector<double> y = {1, 1, 1, 1};

    add_block_multiplication_to_result(b1, x, y);
    add_block_multiplication_to_result(b2, x, y);

    // After b1 on row0: 1 + [1 2 3]·[1 2 3] = 1 + (1 + 4 + 9) = 15
    // After b2 on row3: 1 + [4 5 6]·[1 2 3] = 1 + (4 + 10 + 18) = 33
    ASSERT_EQ(y.size(), 4u);
    EXPECT_NEAR(y[0], 15.0, TOL);
    EXPECT_NEAR(y[1],  1.0, TOL);
    EXPECT_NEAR(y[2],  1.0, TOL);
    EXPECT_NEAR(y[3], 33.0, TOL);
}

TEST(SparseMatrixDetail, AddBlockMul_SameBlockTwice_Doubles) {
    const SparseMatrix::Block b = {
        1, 1, 2, 2,
        { 2.0, -1.0,
          0.5, 3.0 }
    };
    std::vector<double> x = {10, -2, 4, 0, 7};
    std::vector<double> y(3, 0.0);

    add_block_multiplication_to_result(b, x, y);
    add_block_multiplication_to_result(b, x, y);

    // Once:
    // row1: [2 -1]·[-2 4]   = -4 + -4 = -8
    // row2: [0.5 3]·[-2 4]  = -1 + 12 = 11
    // Twice => row1=-16, row2=22
    ASSERT_EQ(y.size(), 3u);
    EXPECT_NEAR(y[0],   0.0, TOL);
    EXPECT_NEAR(y[1], -16.0, TOL);
    EXPECT_NEAR(y[2],  22.0, TOL);
}

// add_transposed_block_multiplication_to_result: z += Bᵀ*x (single block)
TEST(SparseMatrixDetail, AddTransposedBlockMul_BasicCenter) {
    const SparseMatrix::Block b = {
        1, 3, 3, 2,
        { 2.0, 1.0,
          0.0, 3.0,
          4.0, 5.0 }
    };
    std::vector<double> x = {1, 2, 3, 4, 5}; // m=5
    std::vector<double> z(7, 0.0);

    add_transposed_block_multiplication_to_result(b, x, z);

    // Col3: 2*x[1] + 0*x[2] + 4*x[3] = 4 + 0 + 16 = 20
    // Col4: 1*x[1] + 3*x[2] + 5*x[3] = 2 + 9 + 20 = 31
    ASSERT_EQ(z.size(), 7u);
    EXPECT_NEAR(z[0], 0.0, TOL);
    EXPECT_NEAR(z[1], 0.0, TOL);
    EXPECT_NEAR(z[2], 0.0, TOL);
    EXPECT_NEAR(z[3], 20.0, TOL);
    EXPECT_NEAR(z[4], 31.0, TOL);
    EXPECT_NEAR(z[5], 0.0, TOL);
    EXPECT_NEAR(z[6], 0.0, TOL);
}

TEST(SparseMatrixDetail, AddTransposedBlockMul_Accumulates) {
    const SparseMatrix::Block b1 = { 0, 0, 2, 2, {1.0, 2.0, 3.0, 4.0} };
    const SparseMatrix::Block b2 = { 3, 3, 3, 2, {5.0, 6.0, 7.0, 8.0, 9.0, 10.0} };
    std::vector<double> x = {1, 0, 2, -1, 3, 4}; // m=6
    std::vector<double> z(6, -2.0);

    add_transposed_block_multiplication_to_result(b1, x, z);
    add_transposed_block_multiplication_to_result(b2, x, z);

    // b1: j=0,1 ; rows 0,1
    //  z0 += 1*x0 + 3*x1 = 1*1 + 3*0 = 1
    //  z1 += 2*x0 + 4*x1 = 2*1 + 4*0 = 2
    // b2: j=3,4 ; rows 3,4,5
    //  z3 += 5*(-1) + 7*3 + 9*4 = -5 + 21 + 36 = 52
    //  z4 += 6*(-1) + 8*3 + 10*4 = -6 + 24 + 40 = 58
    // Start from -2 each:
    // z = [-1, 0, -2, 50, 56, -2]
    ASSERT_EQ(z.size(), 6u);
    EXPECT_NEAR(z[0], -1.0, TOL);
    EXPECT_NEAR(z[1],  0.0, TOL);
    EXPECT_NEAR(z[2], -2.0, TOL);
    EXPECT_NEAR(z[3], 50.0, TOL);
    EXPECT_NEAR(z[4], 56.0, TOL);
    EXPECT_NEAR(z[5], -2.0, TOL);
}

TEST(SparseMatrixDetail, AddTransposedBlockMul_SameBlockTwice_Doubles) {
    const SparseMatrix::Block b = {
        2, 1, 2, 3,
        { 1.0, -2.0, 0.5,
          3.0,  4.0, 1.0 }
    };
    std::vector<double> x = {0.0, 2.0, -1.0, 3.0, 5.0}; // m=5
    std::vector<double> z(5, 0.0);

    add_transposed_block_multiplication_to_result(b, x, z);
    add_transposed_block_multiplication_to_result(b, x, z);

    // Once:
    //  z1 += 1*x2 + 3*x3 = (-1) + 9 = 8
    //  z2 += -2*x2 + 4*x3 =  2  + 12 = 14
    //  z3 += 0.5*x2 + 1*x3 = -0.5 + 3 = 2.5
    // Twice: [0,16,28,5,0]
    ASSERT_EQ(z.size(), 5u);
    EXPECT_NEAR(z[0],  0.0, TOL);
    EXPECT_NEAR(z[1], 16.0, TOL);
    EXPECT_NEAR(z[2], 28.0, TOL);
    EXPECT_NEAR(z[3],  5.0, TOL);
    EXPECT_NEAR(z[4],  0.0, TOL);
}

// -------------------------------- is_block_valid --------------------------------

TEST(SparseMatrixDetail, IsBlockValid_ExactFitAtOrigin) {
    SparseMatrix::Block b{0, 0, 2, 3, {}};
    // matrix m=2, n=3
    EXPECT_TRUE(is_block_valid(b, /*n=*/3, /*m=*/2));
}

TEST(SparseMatrixDetail, IsBlockValid_ExactFitAtEnd) {
    SparseMatrix::Block b{3, 5, 2, 2, {}};
    // matrix m=5, n=7 ; i+ilen=5, j+jlen=7
    EXPECT_TRUE(is_block_valid(b, /*n=*/7, /*m=*/5));
}

TEST(SparseMatrixDetail, IsBlockValid_OffByOnePastRow) {
    SparseMatrix::Block b{4, 0, 2, 1, {}};
    // m=5 => 4+2=6 > 5 -> false
    EXPECT_FALSE(is_block_valid(b, /*n=*/3, /*m=*/5));
}

TEST(SparseMatrixDetail, IsBlockValid_OffByOnePastCol) {
    SparseMatrix::Block b{0, 6, 1, 2, {}};
    // n=7 => 6+2=8 > 7 -> false
    EXPECT_FALSE(is_block_valid(b, /*n=*/7, /*m=*/3));
}

TEST(SparseMatrixDetail, IsBlockValid_NegativeIndicesOrZeroLengths) {
    EXPECT_FALSE(is_block_valid(SparseMatrix::Block{-1, 0, 1, 1, {}}, 3, 3));
    EXPECT_FALSE(is_block_valid(SparseMatrix::Block{0, -2, 1, 1, {}}, 3, 3));
    EXPECT_FALSE(is_block_valid(SparseMatrix::Block{0, 0, 0, 1, {}}, 3, 3));
    EXPECT_FALSE(is_block_valid(SparseMatrix::Block{0, 0, 1, 0, {}}, 3, 3));
}

TEST(SparseMatrixDetail, IsBlockValid_Inside) {
    EXPECT_TRUE(is_block_valid(SparseMatrix::Block{2, 3, 3, 4, {}}, 8, 10));
}