#pragma once
#include <vector>
#include <span>
#include "sparse_matrix.hpp"
#include "particle.hpp"


class Constraint {

public:
    virtual std::vector<SparseMatrix::Block> create_j_blocks() = 0;
    virtual std::vector<SparseMatrix::Block> create_j_dot_blocks() = 0;

    virtual void update_j_blocks(const std::vector<Particle>& particles, std::span<SparseMatrix::Block> j_blocks) = 0;
    virtual void update_j_dot_blocks(const std::vector<Particle>& particles, std::span<SparseMatrix::Block> j_dot_blocks) = 0;
    virtual double return_c(const std::vector<Particle>& particles) = 0;
    virtual double return_c_dot(const std::vector<Particle>& particles) = 0;
};