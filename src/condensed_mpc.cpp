#include "affine_mpc/condensed_mpc.hpp"

#include "affine_mpc/mpc_base.hpp"

namespace affine_mpc {


CondensedMPC::CondensedMPC(int state_dim,
                           int input_dim,
                           const Parameterization& param,
                           const Options& opts) :
    MPCBase(
        state_dim, input_dim, param, opts, detail::FormulationKind::Condensed)
{}

CondensedMPC::CondensedMPC(int state_dim,
                           int input_dim,
                           int horizon_steps,
                           const Options& opts) :
    CondensedMPC(state_dim,
                 input_dim,
                 Parameterization::moveBlocking(horizon_steps, horizon_steps),
                 opts)
{
  // nothing to do, delegating to main constructor with no parameterization
}

} // namespace affine_mpc
