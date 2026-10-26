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

namespace affine_mpc::detail {

/**
 * @struct QP
 * @brief Internal: the QP
 *   minimize 1/2 z^T P z + q^T z subject to l <= A z <= u,
 *   in OSQP's form. Owned by the QP builder, which sizes it once at
 *   construction; the formulation and the QP terms write it.
 */
struct QP
{
  Eigen::MatrixXd P; ///< cost matrix
  Eigen::MatrixXd A; ///< constraint matrix
  Eigen::VectorXd q; ///< cost vector
  Eigen::VectorXd l; ///< constraint lower bounds
  Eigen::VectorXd u; ///< constraint upper bounds
};

} // namespace affine_mpc::detail

#endif // AFFINE_MPC_DETAIL_QP_HPP
