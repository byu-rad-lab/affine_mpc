#ifndef MPC_CONFIG_HPP
#define MPC_CONFIG_HPP

namespace affine_mpc {

struct Options
{
  bool use_input_cost = false;
  bool slew_initial_input = false;
  bool slew_control_points = false;
  bool saturate_states = false;
};

} // namespace affine_mpc

#endif // MPC_CONFIG_HPP
