#include "engine/constraint_solver.hpp"
#include "engine/detail/constraint_solver.hpp"

ConstraintSolver::ConstraintSolver(std::vector<std::unique_ptr<Constraint>>&& constraints, int particle_count, double spring_constant, double damping_constant)
    : constraints(std::move(constraints)), 
        c(this->constraints.size()),
        c_dot(this->constraints.size()),
        j(SparseMatrix(this->constraints.size(), particle_count * DIMENSION_COUNT)),
        j_dot(SparseMatrix(this->constraints.size(), particle_count * DIMENSION_COUNT)),
        spring_constant(spring_constant),
        damping_constant(damping_constant) {

    for (const std::unique_ptr<Constraint>& constraint : this->constraints) {
        j_constraint_starts.push_back(j.blocks.size());
        j_dot_constraint_starts.push_back(j.blocks.size());
        std::vector<SparseMatrix::Block> j_blocks = constraint->create_j_blocks();
        std::vector<SparseMatrix::Block> j_dot_blocks = constraint->create_j_dot_blocks();
        for (SparseMatrix::Block& j_block : j_blocks) {
            j.blocks.push_back(std::move(j_block));
        }
        for (SparseMatrix::Block& j_dot_block : j_dot_blocks) {
            j_dot.blocks.push_back(std::move(j_dot_block));
        }
    }

}

void ConstraintSolver::solve(std::vector<Particle>& particles) {
    std::span<SparseMatrix::Block> j_span(j.blocks);
    std::span<SparseMatrix::Block> j_dot_span(j_dot.blocks);
    // Figure out C, C_dot, J, J_dot
    for (int i = 0; i < constraints.size(); i++) {
        std::unique_ptr<Constraint>& constraint = constraints[i];
        c[i] = constraint->return_c(particles);
        c_dot[i] = constraint->return_c_dot(particles);
        std::span<SparseMatrix::Block> j_subspan = get_block_subspan(j_span, j_constraint_starts, i);
        constraint->update_j_blocks(particles, j_span);
        std::span<SparseMatrix::Block> j_dot_subspan = get_block_subspan(j_dot_span, j_dot_constraint_starts, i);
        constraint->update_j_blocks(particles, j_span);
    }
    // Solve J W J^{T} \lambda = - J_dot q_dot - J W Q for \lambda
    std::vector<double> WQ(DIMENSION_COUNT * particles.size());
    for (int i = 0; i < particles.size(); i++) {
        WQ[2*i]   = particles[i].force_accumulator.x / particles[i].mass;
        WQ[2*i+1] = particles[i].force_accumulator.y / particles[i].mass;
    }
    std::vector<double> JWQ = j.right_multiply_with_vector(WQ);
    std::vector<double> q_dot(DIMENSION_COUNT * particles.size());
    for (int i = 0; i < particles.size(); i++) {
        q_dot[2*i] = particles[i].velocity.x;
        q_dot[2*i+1] = particles[i].velocity.y;
    }
    std::vector<double> J_dot_q_dot = j_dot.right_multiply_with_vector(q_dot);
    std::vector<double> b(constraints.size());
    for (int i = 0; i < b.size(); i++) {
        b[i] = -J_dot_q_dot[i] - JWQ[i] - spring_constant * c[i] - damping_constant * c_dot[i];
    }
    // b = - J_dot q_dot - JWQ - k_s C - k_d C_dot
    // x -> (JWJ^T)x
    auto right_multiply = [&] (const std::vector<double>& x) {
        std::vector<double> WJtx = j.right_multiply_transpose_with_vector(x);
        for (int i = 0; i < particles.size(); i++) {
            WJtx[DIMENSION_COUNT*i] /= particles[i].mass;
            WJtx[DIMENSION_COUNT*i+1] /= particles[i].mass;
        }
        return j.right_multiply_with_vector(WJtx);
    };
    std::vector<double> lambda                  = biconjugate_gradient_method_for_symmetric_matrix(right_multiply, b);
    std::vector<double> global_constraint_force = j.right_multiply_transpose_with_vector(lambda);
    for (int i = 0; i < particles.size(); i++) {
        particles[i].force_accumulator.x += global_constraint_force[2*i];
        particles[i].force_accumulator.y += global_constraint_force[2*i+1];
    }
}

constexpr static double TOL = 1e-9;

// Right multiply should be a function that maps x -> Ax, where A is a symmetric matrix. Solves Ax = b for x
// NOTE: For my use case, A is positive-semidefinite, so although the function would work with non-positive semidefinite matrices, 
// I added some assertions for sanity-check, if used elsewhere, remove them
//template<class T>
std::vector<double> biconjugate_gradient_method_for_symmetric_matrix(
    const std::function<std::vector<double>(const std::vector<double>&)>& right_multiply, const std::vector<double>& b
) {
    const int n = b.size();
    std::vector<double> x(n, 0.0);
    std::vector<double> Ax = right_multiply(x);
    assert(Ax.size() == n);
    std::vector<double> r(n);
    for (int i = 0; i < n; i++) {
        r[i] = b[i] - Ax[i];
    }
    double norm_b = sqrt(vector_norm_squared(b));
    std::vector<double> p = r;
    double last_r_norm_squared = vector_norm_squared(r);
    for (int iter_count = 0; sqrt(last_r_norm_squared) >= TOL * (1 + norm_b); iter_count++) { 
        assert(iter_count <= n);
        const std::vector<double> Ap = right_multiply(p);
        const double ptAp = vector_dot(p, Ap);
        assert(ptAp > TOL); // Note that it's also not supposed to be negative as A is required to be 
        const double alpha = last_r_norm_squared / ptAp;
        for (int i = 0; i < n; i++) {
            x[i] += alpha * p[i];
        }
        for (int i = 0; i < n; i++) {
            r[i] -= alpha * Ap[i];
        }
        const double new_r_norm_squared = vector_norm_squared(r);
        const double beta = new_r_norm_squared / last_r_norm_squared;
        for (int i = 0; i < n; i++) {
            p[i] = r[i] + beta * p[i];
        }
        last_r_norm_squared = new_r_norm_squared;
    }
    return x;
}

std::span<SparseMatrix::Block> get_block_subspan(std::span<SparseMatrix::Block> blocks, std::vector<int>& constraint_starts, int constraint_index) {
    int l = constraint_starts[constraint_index];
    int r = constraint_index < (constraint_starts.size() - 1) ? constraint_starts[constraint_index+1] : blocks.size();
    return blocks.subspan(l, r - l);
}

double vector_norm_squared(const std::vector<double>& v) {
    return vector_dot(v, v);
}

double vector_dot(const std::vector<double>& a, const std::vector<double>& b) {
    assert(a.size() == b.size());
    double result = 0;
    for (int i = 0; i < a.size(); i++) {
        result += a[i] * b[i];
    }
    return result;
}