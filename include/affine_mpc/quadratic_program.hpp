#ifndef AFFINE_MPC_QUADRATIC_PROGRAM_HPP
#define AFFINE_MPC_QUADRATIC_PROGRAM_HPP

/**
 * @file quadratic_program.hpp
 * @brief Defines the QuadraticProgram and QPBlock types returned by the MPC's
 *   QP getters.
 */

#include <Eigen/Core>
#include <Eigen/SparseCore>
#include <string>

namespace affine_mpc {

/**
 * @struct QuadraticProgram
 * @brief The QP an MPC solves,
 *   minimize 1/2 z^T P z + q^T z subject to l <= A z <= u,
 *   for analysis or for handing to another solver.
 *
 * P and A are compressed sparse column matrices holding the nonzero entries
 * of the QP, in the same form for every formulation; P is full symmetric
 * (both triangles). The cost omits the constant terms of the MPC cost, so
 * its value at the solution is not the full MPC cost. Equality rows have
 * l = u, and unbounded sides are +-infinity (the solver receives them as its
 * own infinity). The order of the variables z and of the rows of A is given
 * by MPCBase::getQPVariableBlocks() and MPCBase::getQPConstraintBlocks().
 */
struct QuadraticProgram
{
  Eigen::SparseMatrix<double> P; ///< cost matrix (full symmetric)
  Eigen::SparseMatrix<double> A; ///< constraint matrix
  Eigen::VectorXd q;             ///< cost vector
  Eigen::VectorXd l;             ///< constraint lower bounds
  Eigen::VectorXd u;             ///< constraint upper bounds
};

/**
 * @struct QPBlock
 * @brief A named block of QP variables or constraint rows: indices offset,
 *   ..., offset + size - 1.
 */
struct QPBlock
{
  std::string name; ///< e.g. "control_points" or "input_bounds"
  int offset{0};    ///< index of the first variable or row
  int size{0};      ///< number of variables or rows
};

} // namespace affine_mpc

#endif // AFFINE_MPC_QUADRATIC_PROGRAM_HPP
