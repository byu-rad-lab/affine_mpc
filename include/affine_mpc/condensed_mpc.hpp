#ifndef CONDENSED_MPC_HPP
#define CONDENSED_MPC_HPP

#include <Eigen/Core>

#include "affine_mpc/mpc_base.hpp"

namespace affine_mpc {

class CondensedMPC : public MPCBase
{
public:
  CondensedMPC(const int state_dim,
               const int input_dim,
               const int horizon_steps,
               const int num_control_points,
               const int spline_degree,
               const Eigen::Ref<const Eigen::VectorXd>& spline_knots =
                   Eigen::VectorXd(0),
               const bool use_input_cost = false,
               const bool use_slew_rate = false,
               const bool saturate_states = false);
  virtual ~CondensedMPC() = default;

  void getPredictedStateTrajectory(
      Eigen::Ref<Eigen::VectorXd> x_traj) const noexcept override final;

protected: // for testing
  void qpUpdateX0(const Eigen::Ref<const Eigen::VectorXd>& x0) override final;

  void updateS();
  void updateV(const Eigen::Ref<const Eigen::VectorXd>& x0);

  Eigen::MatrixXd S_;
  Eigen::VectorXd v_;

private:
  bool qpUpdateModel() override final;
  bool qpUpdateReferences() override final;
  bool qpUpdateInputLimits() override final;
  bool qpUpdateStateLimits() override final;
  bool qpUpdateSlewRate() override final;

  bool model_changed_;
};

} // namespace affine_mpc

#endif // CONDENSED_MPC_HPP
