#include "engine/sparse_matrix.hpp"
#include <cassert>
#include "engine/detail/sparse_matrix.hpp"

SparseMatrix::SparseMatrix(int m, int n) : m(m), n(n) {}

std::vector<double> SparseMatrix::right_multiply_with_vector(const std::vector<double>& v)const {
    assert(v.size() == static_cast<size_t>(n));
    std::vector<double> result(m);
    for (const SparseMatrix::Block& block : blocks) {
        add_block_multiplication_to_result(block, v, result);
    }
    return result;
}

std::vector<double> SparseMatrix::right_multiply_transpose_with_vector(const std::vector<double>& v)const {
    assert(v.size() == static_cast<size_t>(m));
    std::vector<double> result(n);
    for (const SparseMatrix::Block& block : blocks) {
        add_transposed_block_multiplication_to_result(block, v, result);
    }
    return result;
}

void add_block_multiplication_to_result(const SparseMatrix::Block& block, const std::vector<double>& v, std::vector<double>& result) {
    assert(is_block_valid(block, v.size(), result.size()));
    for (int i = block.i; i < block.i + block.ilength; i++) {
        for (int j = block.j; j < block.j + block.jlength; j++) {
            result[i] += v[j] * block.data[(i - block.i) * block.jlength + j - block.j];
        }
    }
}

void add_transposed_block_multiplication_to_result(const SparseMatrix::Block& block, const std::vector<double>& v, std::vector<double>& result) {
    assert(is_block_valid(block, result.size(), v.size()));
    for (int i = block.i; i < block.i + block.ilength; i++) {
        for (int j = block.j; j < block.j + block.jlength; j++) {
            result[j] += v[i] * block.data[(i - block.i) * block.jlength + j - block.j];
        }
    }
}

bool is_block_valid(const SparseMatrix::Block& block, int n, int m) {
    return 0 <= block.i &&
           0 <= block.j && 
           0 < block.ilength && 
           0 < block.jlength &&
           block.i + block.ilength <= m &&
           block.j + block.jlength <= n;
}