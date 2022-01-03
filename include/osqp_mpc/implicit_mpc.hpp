#ifndef IMPLICIT_MPC_HPP
#define IMPLICIT_MPC_HPP

#include <Eigen/Core>
#include <fstream>
#include "osqp_mpc/osqp_solver.hpp"
#include "osqp_mpc/mpc_base.hpp"

using namespace Eigen;

class ImplicitMPC : public MPCBase
{
public:
  ImplicitMPC(const int num_states, const int num_inputs, const int horizon_length,
              const int num_knot_points, const bool use_input_cost=false,
              const bool use_slew_rate=false, const bool saturate_states=false);
  virtual ~ImplicitMPC();

  bool calcNextInput(const Ref<const VectorXd>& x0, Ref<VectorXd> u) override;
  bool calcInputTrajectory(const Ref<const VectorXd>& x0, Ref<VectorXd> u_traj) override;
  void initSolver(const OSQPSettings* solver_settings = nullptr);

  void setInputLimits(const Ref<const VectorXd>& u_min, const Ref<const VectorXd>& u_max);
  void setStateLimits(const Ref<const VectorXd>& x_min, const Ref<const VectorXd>& x_max);
  void setSlewRate(const Ref<const VectorXd>& u_slew);

protected:
  bool solve(const Ref<const VectorXd>& x0);
  void convertToQP(const Ref<const VectorXd>& x0);
  void calcSAndV(const Ref<const VectorXd>& x0);
  void calcPandQ();

  const int x_sat_idx_;
  const int num_constraints_;
  bool initialized_;
  OSQPSolver solver_;
  MatrixXd P_; // osqp cost matrix
  MatrixXd A_; // osqp constraint matrix
  MatrixXd S_;
  VectorXd v_;
  VectorXd q_; // osqp cost vector
  VectorXd l_; // osqp constraint lower bound
  VectorXd u_; // osqp constraint upper bound
};

#endif // IMPLICIT_MPC_HPP
