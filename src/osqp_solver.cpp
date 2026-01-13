#include "affine_mpc/osqp_solver.hpp"

#include <Eigen/Core>
#include <cassert>

namespace affine_mpc {

OSQPSolver::OSQPSolver(const int num_variables, const int num_constraints) :
    settings_{(OSQPSettings*)c_malloc(sizeof(OSQPSettings))},
    data_{(OSQPData*)c_malloc(sizeof(OSQPData))},
    workspace_initialized_{false},
    P_is_set_{false},
    A_is_set_{false},
    n_{num_variables},
    m_{num_constraints},
    P_p_{num_variables+1},
    A_p_{num_variables+1}
{
  assert(num_variables > 0 && num_constraints > 0);
  osqp_set_default_settings(settings_);
  settings_->alpha = 1.0;
  settings_->verbose = false;
  settings_->eps_dual_inf = 1e-6;
  settings_->eps_prim_inf = 1e-6;
}

OSQPSolver::~OSQPSolver()
{
  if (data_)
  {
    if (A_is_set_) c_free(data_->A);
    if (P_is_set_) c_free(data_->P);
    c_free(data_);
  }
  if (settings_) c_free(settings_);
}

const c_float* OSQPSolver::getSolutionPtr() const
{
  if (workspace_initialized_)
    return work_->solution->x;
  else
    return nullptr;
}

c_float OSQPSolver::getSolveTime() const
{
  return work_->info->solve_time;
}

bool OSQPSolver::solve(Eigen::Ref<VectorXF> solution)
{
  assert(solution.size() == n_);
  assert(workspace_initialized_);

  osqp_solve(work_);
  Eigen::Map<VectorXF> map_solution{work_->solution->x, n_, 1};
  solution = map_solution;
  return work_->info->status_val == OSQP_SOLVED;
}

bool OSQPSolver::solve()
{
  assert(workspace_initialized_);
  osqp_solve(work_);
  return work_->info->status_val == OSQP_SOLVED;
}

bool OSQPSolver::initialize(const Eigen::Ref<const MatrixXF>& P, const Eigen::Ref<const MatrixXF>& A,
                            Eigen::Ref<VectorXF> q, Eigen::Ref<VectorXF> l, Eigen::Ref<VectorXF> u,
                            const OSQPSettings* settings)
{
  assert(q.size() == n_ && l.size() == m_ && u.size() == m_);
  if (!data_ || !settings_) return false;

  if (settings)
    setCustomSettings(settings);
  data_->n = n_;
  data_->m = m_;
  initializeCostMatrix(P);
  initializeConstraintMatrix(A);
  data_->q = q.data();
  data_->l = l.data();
  data_->u = u.data();

  c_int exitflag{osqp_setup(&work_, data_, settings_)};
  if (exitflag == 0)
    workspace_initialized_ = true;
  return workspace_initialized_;
}

bool OSQPSolver::updateCostMatrix(const Eigen::Ref<const MatrixXF>& P)
{
  if (!workspace_initialized_) return false;
  assert(P.rows() == n_ && P.cols() == n_);

  int idx{0}, idx_diff, row;
  for (int col{0}; col < n_; ++col)
  {
    idx_diff = P_p_(col+1) - P_p_(col);
    while (idx_diff != 0)
    {
      row = P_i_(idx);
      P_x_(idx++) = P(row,col);
      --idx_diff;
    }
  }
  c_int exit_status{osqp_update_P(work_, P_x_.data(), OSQP_NULL, P_nnz_)};
  return exit_status == 0;
}

bool OSQPSolver::updateConstraintMatrix(const Eigen::Ref<const MatrixXF>& A)
{
  if (!workspace_initialized_) return false;
  assert(A.rows() == m_ && A.cols() == n_);
  assert(A.count() <= A_nnz_ && "A cannot change structure once initialized");

  int idx{0}, idx_diff, row;
  for (int col{0}; col < n_; ++col)
  {
    idx_diff = A_p_(col+1) - A_p_(col);
    while (idx_diff != 0)
    {
      row = A_i_(idx);
      A_x_(idx++) = A(row,col);
      --idx_diff;
    }
  }
  c_int exit_status{osqp_update_A(work_, A_x_.data(), OSQP_NULL, A_nnz_)};
  return exit_status == 0;
}

bool OSQPSolver::updateCostVector(Eigen::Ref<VectorXF> q)
{
  assert(q.size() == n_);
  if (!workspace_initialized_) return false;

  c_int exit_status{osqp_update_lin_cost(work_, q.data())};
  return exit_status == 0;
}

bool OSQPSolver::updateBounds(Eigen::Ref<VectorXF> l, Eigen::Ref<VectorXF> u)
{
  assert(l.size() == m_ && u.size() == m_);
  if (!workspace_initialized_) return false;

  c_int exit_status{osqp_update_bounds(work_, l.data(), u.data())};
  return exit_status == 0;
}

int OSQPSolver::countUpperTriangle(const Eigen::Ref<const MatrixXF>& mat)
{
  int count{0};
  for (int col{0}; col < n_; ++col)
  {
    for (int row{0}; row < n_; ++row)
    {
      if (row > col) break;
      if (mat(row,col) != 0.0)
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
  c_float val;
  for (int col{0}; col < n_; ++col)
  {
    P_p_(col) = P_nnz_;
    for (int row{0}; row < n_; ++row)
    {
      if (row > col) break;
      val = P(row,col);
      if (val != 0.0)
      {
        P_x_(P_nnz_) = val;
        P_i_(P_nnz_++) = row;
      }
    }
  }
  P_p_(n_) = P_nnz_;
  data_->P = csc_matrix(n_, n_, P_nnz_, P_x_.data(), P_i_.data(), P_p_.data());
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
  c_float val;
  for (int col{0}; col < n_; ++col)
  {
    A_p_(col) = A_nnz_;
    for (int row{0}; row < m_; ++row)
    {
      val = A(row,col);
      if (val != 0.0)
      {
        A_x_(A_nnz_) = val;
        A_i_(A_nnz_++) = row;
      }
    }
  }
  A_p_(n_) = A_nnz_;
  data_->A = csc_matrix(m_, n_, A_nnz_, A_x_.data(), A_i_.data(), A_p_.data());
  A_is_set_ = true;
}

void OSQPSolver::setCustomSettings(const OSQPSettings* settings)
{
  settings_->rho = settings->rho;
  settings_->sigma = settings->sigma;
  settings_->max_iter = settings->max_iter;
  settings_->eps_abs = settings->eps_abs;
  settings_->eps_rel = settings->eps_rel;
  settings_->eps_prim_inf = settings->eps_prim_inf;
  settings_->eps_dual_inf = settings->eps_dual_inf;
  settings_->alpha = settings->alpha;
  settings_->linsys_solver = settings->linsys_solver;
  settings_->delta = settings->delta;
  settings_->polish = settings->polish;
  settings_->polish_refine_iter = settings->polish_refine_iter;
  settings_->verbose = settings->verbose;
  settings_->scaled_termination = settings->scaled_termination;
  settings_->check_termination = settings->check_termination;
  settings_->warm_start = settings->warm_start;
  settings_->scaling = settings->scaling;
  settings_->adaptive_rho = settings->adaptive_rho;
  settings_->adaptive_rho_interval = settings->adaptive_rho_interval;
  settings_->adaptive_rho_tolerance = settings->adaptive_rho_tolerance;
  settings_->adaptive_rho_fraction = settings->adaptive_rho_fraction;
  settings_->time_limit = settings->time_limit;
}

} // namespace affine_mpc
