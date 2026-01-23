#ifndef IMPLICIT_MPC_HPP
#define IMPLICIT_MPC_HPP

#include <Eigen/Core>

#include "affine_mpc/mpc_base.hpp"

namespace affine_mpc {


class ImplicitMPC : public MPCBase
{
public:
  ImplicitMPC(const int num_states,
              const int num_inputs,
              const int horizon_length,
              const int num_control_points,
              const bool use_input_cost = false,
              const bool use_slew_rate = false,
              const bool saturate_states = false);
  virtual ~ImplicitMPC() = default;

  // See https://arxiv.org/pdf/2001.04931 section IV.C for details on evaluating
  // the parameterized input trajectory to get the input trajectory
  void getInputTrajectory(Eigen::Ref<Eigen::VectorXd> u_traj) const override;
  void getPredictedStateTrajectory(
      Eigen::Ref<Eigen::VectorXd> x_traj) const override;

  void setInputLimits(const Eigen::Ref<const Eigen::VectorXd>& u_min,
                      const Eigen::Ref<const Eigen::VectorXd>& u_max);
  void setStateLimits(const Eigen::Ref<const Eigen::VectorXd>& x_min,
                      const Eigen::Ref<const Eigen::VectorXd>& x_max);
  void setSlewRate(const Eigen::Ref<const Eigen::VectorXd>& u_slew);

protected:
  // See https://arxiv.org/pdf/2001.04931 sections IV.B and IV.E for details on
  // S and v and how to convert to a QP problem
  void convertToQP(const Eigen::Ref<const Eigen::VectorXd>& x0) override;
  void calcSAndV(const Eigen::Ref<const Eigen::VectorXd>& x0);
  void calcPandQ();

  const int x_sat_idx_;
  const int num_constraints_;
  Eigen::MatrixXd S_;
  Eigen::VectorXd v_;
};

} // namespace affine_mpc

#endif // IMPLICIT_MPC_HPP
