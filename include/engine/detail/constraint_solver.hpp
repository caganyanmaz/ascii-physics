#pragma once
#include <functional>

constexpr static int DIMENSION_COUNT = 2;
std::span<SparseMatrix::Block> get_block_subspan(std::span<SparseMatrix::Block> blocks, std::vector<int>& constraint_starts, int constraint_index);
//template<class T>
std::vector<double> biconjugate_gradient_method_for_symmetric_matrix(const std::function<std::vector<double>(const std::vector<double>&)>& right_multiply, const std::vector<double>& b);

double vector_dot(const std::vector<double>& a, const std::vector<double>& b);
double vector_norm_squared(const std::vector<double>& v);

