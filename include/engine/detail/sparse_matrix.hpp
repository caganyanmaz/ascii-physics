#pragma once
#include "engine/sparse_matrix.hpp"

void add_block_multiplication_to_result(const SparseMatrix::Block& block, const std::vector<double>& v, std::vector<double>& result);
void add_transposed_block_multiplication_to_result(const SparseMatrix::Block& block, const std::vector<double>& v, std::vector<double>& result);
bool is_block_valid(const SparseMatrix::Block& block, int n, int m);