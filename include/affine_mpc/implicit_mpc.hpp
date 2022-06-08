#ifndef IMPLICIT_MPC_HPP
#define IMPLICIT_MPC_HPP

#include <Eigen/Core>
#include "affine_mpc/mpc_base.hpp"

using namespace Eigen;

class ImplicitMPC : public MPCBase
{
public:
  ImplicitMPC(const int num_states, const int num_inputs, const int horizon_length,
              const int num_knot_points, const bool use_input_cost=false,
              const bool use_slew_rate=false, const bool saturate_states=false);
  virtual ~ImplicitMPC() = default;

  void getPredictedStateTrajectory(Ref<VectorXd> x_traj) const override;

  void setInputLimits(const Ref<const VectorXd>& u_min, const Ref<const VectorXd>& u_max);
  void setStateLimits(const Ref<const VectorXd>& x_min, const Ref<const VectorXd>& x_max);
  void setSlewRate(const Ref<const VectorXd>& u_slew);

protected:
  void convertToQP(const Ref<const VectorXd>& x0) override;
  void calcSAndV(const Ref<const VectorXd>& x0);
  void calcPandQ();

  const int x_sat_idx_;
  const int num_constraints_;
  MatrixXd S_;
  VectorXd v_;
};

#endif // IMPLICIT_MPC_HPP
