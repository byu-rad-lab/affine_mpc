#include "affine_mpc/detail/vector_parameterization.hpp"

#include <Eigen/Core>
#include <cassert>
#include <vector>

#include "affine_mpc/parameterization.hpp"

using namespace Eigen;

namespace affine_mpc::detail {

VectorParameterization::VectorParameterization(int input_dim,
                                               const Parameterization& param) :
    param_{param},
    input_dim_{input_dim},
    num_coefficients_{input_dim * param.num_control_points},
    offsets_{VectorXi::LinSpaced(input_dim, 0, input_dim - 1)},
    strides_{VectorXi::Constant(input_dim, input_dim)}
{
  assert(input_dim > 0);
}

// Both evaluations rely on the shared interleaved layout: the coefficients
// form an input_dim x num_control_points matrix with one control point per
// column.
void VectorParameterization::evaluate(const int k,
                                      const Ref<const VectorXd>& coefficients,
                                      Ref<VectorXd> uk) const noexcept
{
  const Map<const MatrixXd> ctrls{coefficients.data(), input_dim_,
                                  param_.num_control_points};
  uk.noalias() = ctrls.middleCols(param_.band_start(k), param_.band_width)
                 * param_.band_weights.col(k);
}

void VectorParameterization::evaluateTrajectory(
    const Ref<const VectorXd>& coefficients,
    Ref<VectorXd> u_traj) const noexcept
{
  const Map<const MatrixXd> ctrls{coefficients.data(), input_dim_,
                                  param_.num_control_points};
  Map<MatrixXd> u_mat{u_traj.data(), input_dim_, param_.horizon_steps};
  for (int k{0}; k < param_.horizon_steps; ++k) {
    u_mat.col(k).noalias() =
        ctrls.middleCols(param_.band_start(k), param_.band_width)
        * param_.band_weights.col(k);
  }
}

std::vector<int> VectorParameterization::inputSaturationSamples() const
{
  return param_.inputSaturationSamples();
}

} // namespace affine_mpc::detail
