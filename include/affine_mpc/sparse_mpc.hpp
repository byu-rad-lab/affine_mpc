#ifndef SPARSE_MPC_HPP
#define SPARSE_MPC_HPP

#include <Eigen/Core>

#include "affine_mpc/mpc_base.hpp"

namespace affine_mpc {

class SparseMPC : public MPCBase
{
public:
  SparseMPC(const int state_dim,
            const int input_dim,
            const int horizon_steps,
            const int num_control_points,
            const int spline_degree,
            const Eigen::Ref<const Eigen::VectorXd>& spline_knots =
                Eigen::VectorXd(0),
            const bool use_input_cost = false,
            const bool use_slew_rate = false,
            const bool saturate_states = false);
  virtual ~SparseMPC() = default;

  void getInputTrajectory(
      Eigen::Ref<Eigen::VectorXd> u_traj) const noexcept override final
  {
    MPCBase::getInputTrajectory(u_traj);
  }
  void getPredictedStateTrajectory(
      Eigen::Ref<Eigen::VectorXd> x_traj) const noexcept override final;

  bool setModelDiscrete(const Eigen::Ref<const Eigen::MatrixXd>& Ad,
                        const Eigen::Ref<const Eigen::MatrixXd>& Bd,
                        const Eigen::Ref<const Eigen::VectorXd>& wd);
  bool setModelContinuous2Discrete(const Eigen::Ref<const Eigen::MatrixXd>& Ac,
                                   const Eigen::Ref<const Eigen::MatrixXd>& Bc,
                                   const Eigen::Ref<const Eigen::VectorXd>& wc,
                                   double dt,
                                   double tol = 1e-6);

  void setReferenceState(const Eigen::Ref<const Eigen::VectorXd>& x_step);
  void setReferenceInput(const Eigen::Ref<const Eigen::VectorXd>& u_step);
  void
  setReferenceStateTrajectory(const Eigen::Ref<const Eigen::VectorXd>& x_traj);
  void setReferenceParameterizedInputTrajectory(
      const Eigen::Ref<const Eigen::VectorXd>& u_traj_ctrl_pts);

  void setInputLimits(const Eigen::Ref<const Eigen::VectorXd>& u_min,
                      const Eigen::Ref<const Eigen::VectorXd>& u_max);
  void setStateLimits(const Eigen::Ref<const Eigen::VectorXd>& x_min,
                      const Eigen::Ref<const Eigen::VectorXd>& x_max);
  void setSlewRate(const Eigen::Ref<const Eigen::VectorXd>& u_slew);

protected:
  void updateQP(const Eigen::Ref<const Eigen::VectorXd>& x0) override final;
  void calcBothCostTerms();
  void calcCostVector();
  // void initializeConstraintMatrix();
  bool qpUpdateModel();

  const int x_sat_idx_;
  const int num_constraints_;

private:
  bool refs_changed_;
};

} // namespace affine_mpc

#endif // SPARSE_MPC_HPP
