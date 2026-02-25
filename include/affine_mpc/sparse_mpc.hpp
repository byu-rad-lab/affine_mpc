#ifndef SPARSE_MPC_HPP
#define SPARSE_MPC_HPP

#include <Eigen/Core>

#include "affine_mpc/mpc_base.hpp"
#include "affine_mpc/options.hpp"
#include "affine_mpc/parameterization.hpp"

namespace affine_mpc {

class SparseMPC : public MPCBase
{
public:
  SparseMPC(const int state_dim,
            const int input_dim,
            const Parameterization& param,
            const Options& opts = Options{});
  SparseMPC(const int state_dim,
            const int input_dim,
            const int horizon_steps,
            const Options& opts = Options{});
  virtual ~SparseMPC() = default;

  void getPredictedStateTrajectory(
      Eigen::Ref<Eigen::VectorXd> x_traj) const noexcept override final;

protected: // for testing
  void qpUpdateX0(const Eigen::Ref<const Eigen::VectorXd>& x0) override final;

private:
  bool qpUpdateModel() override final;
  bool qpUpdateReferences() override final;
  bool qpUpdateInputLimits() override final;
  bool qpUpdateStateLimits() override final;
  bool qpUpdateSlewRate() override final;

  void calcBothCostTerms();
  void calcCostVector();

  bool refs_changed_;
};

} // namespace affine_mpc

#endif // SPARSE_MPC_HPP
