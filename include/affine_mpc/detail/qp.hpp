#ifndef AFFINE_MPC_DETAIL_QP_HPP
#define AFFINE_MPC_DETAIL_QP_HPP

/**
 * @file qp.hpp
 * @brief Internal: the QP data passed to the solver.
 *
 * Internal: installed only because public headers include it (MPCBase gives
 * subclasses read access to the QP). Not part of the public API.
 */

#include <Eigen/Core>
#include <Eigen/SparseCore>
#include <osqp.h>

namespace affine_mpc::detail {

/// How the QP stores P and A, chosen by the formulation.
enum class QPStorage
{
  Dense, ///< dense P (full symmetric) and A
  Sparse ///< CSC P (upper triangle) and A
};

/**
 * @struct QP
 * @brief Internal: the QP
 *   minimize 1/2 z^T P z + q^T z subject to l <= A z <= u,
 *   in OSQP's form. Owned by the QP builder, which sizes it once at
 *   construction; the formulation and the QP terms write it.
 *
 * P and A are stored either dense (`P`, `A`; the condensed formulation, whose
 * matrices are genuinely dense) or in compressed sparse column form
 * (`P_csc`, `A_csc`; the sparse formulation), as `storage` says; the members
 * of the other kind are empty. The CSC pattern is fixed at construction (the
 * entries the writers declared), so only its values change afterwards.
 */
struct QP
{
  /// Compressed sparse column matrix in OSQP's layout.
  using SparseMatrix = Eigen::SparseMatrix<double, Eigen::ColMajor, OSQPInt>;

  QPStorage storage{QPStorage::Dense};
  Eigen::MatrixXd P;  ///< dense: cost matrix (full symmetric)
  Eigen::MatrixXd A;  ///< dense: constraint matrix
  SparseMatrix P_csc; ///< sparse: upper triangle of the cost matrix
  SparseMatrix A_csc; ///< sparse: constraint matrix
  Eigen::VectorXd q;  ///< cost vector
  Eigen::VectorXd l;  ///< constraint lower bounds
  Eigen::VectorXd u;  ///< constraint upper bounds
};

/// The full symmetric cost matrix of either storage, written into `P`
/// (resized if needed; not for the control loop).
inline void denseCostMatrix(const QP& qp, Eigen::MatrixXd& P)
{
  if (qp.storage == QPStorage::Dense) {
    P = qp.P;
    return;
  }
  P.setZero(qp.P_csc.rows(), qp.P_csc.cols());
  for (Eigen::Index j{0}; j < qp.P_csc.outerSize(); ++j)
    for (QP::SparseMatrix::InnerIterator it{qp.P_csc, j}; it; ++it) {
      P(it.row(), it.col()) = it.value();
      P(it.col(), it.row()) = it.value();
    }
}

/// The constraint matrix of either storage, written into `A` (resized if
/// needed; not for the control loop).
inline void denseConstraintMatrix(const QP& qp, Eigen::MatrixXd& A)
{
  if (qp.storage == QPStorage::Dense) {
    A = qp.A;
    return;
  }
  A.setZero(qp.A_csc.rows(), qp.A_csc.cols());
  for (Eigen::Index j{0}; j < qp.A_csc.outerSize(); ++j)
    for (QP::SparseMatrix::InnerIterator it{qp.A_csc, j}; it; ++it)
      A(it.row(), it.col()) = it.value();
}

} // namespace affine_mpc::detail

#endif // AFFINE_MPC_DETAIL_QP_HPP
