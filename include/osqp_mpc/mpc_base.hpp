#ifndef MPC_BASE_HPP
#define MPC_BASE_HPP

#include <Eigen/Core>

using namespace Eigen;

class MPCBase
{
public:
  MPCBase(const int num_states, const int num_inputs, const int horizon_length,
          const int num_knot_points, const bool use_input_cost=false,
          const bool use_slew_rate=false, const bool saturate_states=false);
  virtual ~MPCBase();

  virtual bool calcNextInput(const Ref<const VectorXd>& x0, Ref<VectorXd> u) = 0;
  virtual bool calcInputTrajectory(const Ref<const VectorXd>& x0,
                                   Ref<VectorXd> u_traj) = 0;

  void setModelDiscrete(const Ref<const MatrixXd>& Ad,
                        const Ref<const MatrixXd>& Bd,
                        const Ref<const VectorXd>& wd);
  void setModelContinuous2Discrete(const Ref<const MatrixXd>& Ap,
                                   const Ref<const MatrixXd>& Bp,
                                   const Ref<const VectorXd>& wp,
                                   double dt, double tol=1e-6);

  void setWeights(const Ref<const VectorXd>& Q_diag, const Ref<const VectorXd>& R_diag);
  void setStateWeights(const Ref<const VectorXd>& Q_diag);
  void setInputWeights(const Ref<const VectorXd>& R_diag);

  void setDesiredState(const Ref<const VectorXd>& x_step);
  void setDesiredInput(const Ref<const VectorXd>& u_step);
  void setDesiredStateTrajectory(const Ref<const VectorXd>& x_traj);
  void setDesiredInputTrajectory(const Ref<const VectorXd>& u_traj);

  void setInputLimits(const Ref<const VectorXd>& u_min, const Ref<const VectorXd>& u_max);
  void setStateLimits(const Ref<const VectorXd>& x_min, const Ref<const VectorXd>& x_max);
  void setSlewRate(const Ref<const VectorXd>& u_slew);

protected:
  const int n_, m_, T_, p_;
  const bool use_input_cost_, use_slew_rate_, saturate_states_;
  bool model_set_, input_limits_set_, slew_rate_set_, state_limits_set_;
  MatrixXd Ad_;
  MatrixXd Bd_;
  VectorXd wd_;
  DiagonalMatrix<double,Dynamic> Q_big_;
  DiagonalMatrix<double,Dynamic> R_big_;
  VectorXd x_goal_;
  VectorXd u_goal_;
  VectorXd u_min_;
  VectorXd u_max_;
  VectorXd x_min_;
  VectorXd x_max_;
  VectorXd u_slew_;
};

#endif // MPC_BASE_HPP
