#ifndef AFFINE_MPC_OSQP_SOLVER_IMPL_HPP
#define AFFINE_MPC_OSQP_SOLVER_IMPL_HPP

// Internal: the parts of OSQPSolver that use OSQP's types, which stay out of
// the installed headers.

#include <osqp.h>

#include "affine_mpc/osqp_settings.hpp"

namespace affine_mpc {

/// OSQP's settings struct with the values of `settings`.
::OSQPSettings toOSQP(const OSQPSettings& settings) noexcept;

} // namespace affine_mpc

#endif // AFFINE_MPC_OSQP_SOLVER_IMPL_HPP
