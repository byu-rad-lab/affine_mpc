#ifndef BSPLINE_MPC_HPP
#define BSPLINE_MPC_HPP

#include <Eigen/Core>

#include "affine_mpc/mpc_base.hpp"

namespace affine_mpc {

class BSplineMPC : public MPCBase
{
public:
  BSplineMPC(
      const int num_states,
      const int num_inputs,
      const int num_steps,
      const int num_controls,
      const int spline_degree,
      const Eigen::Ref<const Eigen::VectorXd>& knots = Eigen::VectorXd(0),
      const bool use_input_cost = false,
      const bool use_slew_rate = false,
      const bool saturate_states = false);
  using MPCBase::MPCBase;
  virtual ~BSplineMPC() = default;

  void getInputTrajectory(Eigen::Ref<Eigen::VectorXd> u_traj) const override final {
    MPCBase::getInputTrajectory(u_traj);
  }
  void getPredictedStateTrajectory(
      Eigen::Ref<Eigen::VectorXd> x_traj) const override final;

  void setInputLimits(const Eigen::Ref<const Eigen::VectorXd>& u_min,
                      const Eigen::Ref<const Eigen::VectorXd>& u_max);
  void setStateLimits(const Eigen::Ref<const Eigen::VectorXd>& x_min,
                      const Eigen::Ref<const Eigen::VectorXd>& x_max);
  void setSlewRate(const Eigen::Ref<const Eigen::VectorXd>& u_slew);

protected:
  void convertToQP(const Eigen::Ref<const Eigen::VectorXd>& x0) override final;
  void calcSAndV(const Eigen::Ref<const Eigen::VectorXd>& x0);
  void calcPAndQ();

  const int x_sat_idx_;
  const int num_constraints_;
  Eigen::MatrixXd S_;
  Eigen::VectorXd v_;
};

} // namespace affine_mpc

#endif // BSPLINE_MPC_HPP
