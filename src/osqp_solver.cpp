#include "affine_mpc/osqp_solver.hpp"
#include "osqp_api_functions.h"
#include "osqp_api_types.h"

#include <Eigen/Core>
#include <cassert>
#include <memory>

namespace affine_mpc {

OSQPSolver::OSQPSolver(const int num_variables, const int num_constraints) :
    solver_{nullptr, osqp_cleanup},
    // settings_{OSQPSettings_new(), OSQPSettings_free},
    P_{nullptr, OSQPCscMatrix_free},
    A_{nullptr, OSQPCscMatrix_free},
    initialized_{false},
    P_is_set_{false},
    A_is_set_{false},
    n_{num_variables},
    m_{num_constraints},
    P_p_{num_variables + 1},
    A_p_{num_variables + 1}
{
  assert(num_variables > 0 && num_constraints > 0);
}

OSQPSettings OSQPSolver::getDefaultSettings() noexcept
{
  OSQPSettings settings;
  osqp_set_default_settings(&settings);
  return settings;
}

OSQPSettings
OSQPSolver::getRecommendedSettings(const bool polish_near_boundaries) noexcept
{
  OSQPSettings settings;
  osqp_set_default_settings(&settings);
  settings.alpha = 1.0;
  settings.verbose = false;
  settings.eps_dual_inf = 1e-6;
  settings.eps_prim_inf = 1e-6;
  if (polish_near_boundaries)
    settings.polishing = 1;
  return settings;
}

const Eigen::Map<const OSQPSolver::VectorXF>
OSQPSolver::getSolutionMap() const noexcept
{
  if (initialized_)
    return Eigen::Map<const VectorXF>{solver_->solution->x, n_, 1};
  else
    return Eigen::Map<const VectorXF>{nullptr, 0, 1};
}

OSQPFloat OSQPSolver::getSolveTime() const noexcept
{
  return solver_->info->solve_time;
}

bool OSQPSolver::solve(Eigen::Ref<VectorXF> solution)
{
  assert(solution.size() == n_);
  assert(initialized_);

  osqp_solve(solver_.get());
  Eigen::Map<VectorXF> map_solution{solver_->solution->x, n_, 1};
  solution = map_solution;
  return solver_->info->status_val == OSQP_SOLVED;
}

bool OSQPSolver::solve()
{
  assert(initialized_);
  osqp_solve(solver_.get());
  return solver_->info->status_val == OSQP_SOLVED;
}

bool OSQPSolver::initialize(const Eigen::Ref<const MatrixXF>& P,
                            const Eigen::Ref<const MatrixXF>& A,
                            Eigen::Ref<VectorXF> q,
                            Eigen::Ref<VectorXF> l,
                            Eigen::Ref<VectorXF> u,
                            const OSQPSettings& settings)
{
  assert(q.size() == n_ && l.size() == m_ && u.size() == m_);
  initializeCostMatrix(P);
  initializeConstraintMatrix(A);

  ::OSQPSolver* raw_solver = nullptr;
  OSQPInt exitflag{osqp_setup(&raw_solver, P_.get(), q.data(), A_.get(),
                              l.data(), u.data(), m_, n_, &settings)};
  solver_.reset(raw_solver);
  if (exitflag == 0)
    initialized_ = true;
  return initialized_;
}

bool OSQPSolver::updateCostMatrix(const Eigen::Ref<const MatrixXF>& P)
{
  if (!initialized_)
    return false;
  assert(P.rows() == n_ && P.cols() == n_);

  int idx{0}, idx_diff, row;
  for (int col{0}; col < n_; ++col) {
    idx_diff = P_p_(col + 1) - P_p_(col);
    while (idx_diff != 0) {
      row = P_i_(idx);
      P_x_(idx++) = P(row, col);
      --idx_diff;
    }
  }
  OSQPInt exit_status{osqp_update_data_mat(
      solver_.get(), P_x_.data(), OSQP_NULL, P_nnz_, OSQP_NULL, OSQP_NULL, 0)};
  return exit_status == 0;
}

bool OSQPSolver::updateConstraintMatrix(const Eigen::Ref<const MatrixXF>& A)
{
  if (!initialized_)
    return false;
  assert(A.rows() == m_ && A.cols() == n_);
  assert(A.count() <= A_nnz_ && "A cannot change structure once initialized");

  int idx{0}, idx_diff, row;
  for (int col{0}; col < n_; ++col) {
    idx_diff = A_p_(col + 1) - A_p_(col);
    while (idx_diff != 0) {
      row = A_i_(idx);
      A_x_(idx++) = A(row, col);
      --idx_diff;
    }
  }
  OSQPInt exit_status{osqp_update_data_mat(solver_.get(), OSQP_NULL, OSQP_NULL,
                                           0, A_x_.data(), OSQP_NULL, A_nnz_)};
  return exit_status == 0;
}

bool OSQPSolver::updateCostVector(Eigen::Ref<VectorXF> q)
{
  assert(q.size() == n_);
  if (!initialized_)
    return false;

  OSQPInt exit_status{
      osqp_update_data_vec(solver_.get(), q.data(), OSQP_NULL, OSQP_NULL)};
  return exit_status == 0;
}

bool OSQPSolver::updateBounds(Eigen::Ref<VectorXF> l, Eigen::Ref<VectorXF> u)
{
  assert(l.size() == m_ && u.size() == m_);
  if (!initialized_)
    return false;

  OSQPInt exit_status{
      osqp_update_data_vec(solver_.get(), OSQP_NULL, l.data(), u.data())};
  return exit_status == 0;
}

int OSQPSolver::countUpperTriangle(const Eigen::Ref<const MatrixXF>& mat)
{
  int count{0};
  for (int col{0}; col < n_; ++col) {
    for (int row{0}; row < n_; ++row) {
      if (row > col)
        break;
      if (mat(row, col) != 0.0)
        ++count;
    }
  }
  return count;
}

void OSQPSolver::initializeCostMatrix(const Eigen::Ref<const MatrixXF>& P)
{
  assert(!P_is_set_);
  assert(P.rows() == n_ && P.cols() == n_);
  P_nnz_ = countUpperTriangle(P);
  P_x_.setZero(P_nnz_);
  P_i_.setZero(P_nnz_);

  P_nnz_ = 0;
  OSQPFloat val;
  for (int col{0}; col < n_; ++col) {
    P_p_(col) = P_nnz_;
    for (int row{0}; row < n_; ++row) {
      if (row > col)
        break;
      val = P(row, col);
      if (val != 0.0) {
        P_x_(P_nnz_) = val;
        P_i_(P_nnz_++) = row;
      }
    }
  }
  P_p_(n_) = P_nnz_;
  P_.reset(
      OSQPCscMatrix_new(n_, n_, P_nnz_, P_x_.data(), P_i_.data(), P_p_.data()));
  P_is_set_ = true;
}

void OSQPSolver::initializeConstraintMatrix(const Eigen::Ref<const MatrixXF>& A)
{
  assert(!A_is_set_);
  assert(A.rows() == m_ && A.cols() == n_);
  A_nnz_ = A.count();
  A_x_.setZero(A_nnz_);
  A_i_.setZero(A_nnz_);

  A_nnz_ = 0;
  OSQPFloat val;
  for (int col{0}; col < n_; ++col) {
    A_p_(col) = A_nnz_;
    for (int row{0}; row < m_; ++row) {
      val = A(row, col);
      if (val != 0.0) {
        A_x_(A_nnz_) = val;
        A_i_(A_nnz_++) = row;
      }
    }
  }
  A_p_(n_) = A_nnz_;
  A_.reset(
      OSQPCscMatrix_new(m_, n_, A_nnz_, A_x_.data(), A_i_.data(), A_p_.data()));
  A_is_set_ = true;
}

} // namespace affine_mpc
