#include "affine_mpc/sparse_mpc.hpp"

#include "affine_mpc/mpc_base.hpp"

namespace affine_mpc {


SparseMPC::SparseMPC(int state_dim,
                     int input_dim,
                     const Parameterization& param,
                     const Options& opts) :
    MPCBase(state_dim, input_dim, param, opts, detail::FormulationKind::Sparse)
{}

SparseMPC::SparseMPC(int state_dim,
                     int input_dim,
                     int horizon_steps,
                     const Options& opts) :
    SparseMPC(state_dim,
              input_dim,
              Parameterization::moveBlocking(horizon_steps, horizon_steps),
              opts)
{
  // nothing to do, delegating to main constructor with no parameterization
}

} // namespace affine_mpc
