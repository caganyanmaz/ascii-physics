#pragma once
#include <vector>

struct SparseMatrix {
    int m;
    int n;
    struct Block {
        int i;
        int j;
        int ilength;
        int jlength;
        std::vector<double> data;
    };
    SparseMatrix(int m, int n);
    std::vector<Block> blocks;
    std::vector<double> right_multiply_with_vector(const std::vector<double>& v)const;
    std::vector<double> right_multiply_transpose_with_vector(const std::vector<double>& v)const;
};