#include "qp_terms.hpp"

#include <Eigen/Core>
#include <cassert>
#include <vector>

#include "affine_mpc/parameterization.hpp"
#include "formulation.hpp"

using namespace Eigen;

namespace affine_mpc::detail {

namespace {

std::vector<int> boundedSamples(const VectorParameterization& input_param,
                                const bool saturate_input_trajectory)
{
  if (!saturate_input_trajectory)
    return {};
  return input_param.inputSaturationSamples();
}

} // namespace

// ---- StateCost --------------------------------------------------------------

void StateCost::updateMatrix(const MPCData& d, Formulation& f, QP&)
{
  f.addStateQuadraticMatrix(d.Q_big.diagonal());
}

void StateCost::updateVector(const MPCData& d, Formulation& f, QP&)
{
  f.addStateQuadraticVector(d.Q_big.diagonal(), d.x_ref);
}

// ---- InputCost --------------------------------------------------------------

void InputCost::updateMatrix(const MPCData& d, Formulation& f, QP&)
{
  f.addControlQuadraticMatrix(d.R_big.diagonal());
}

void InputCost::updateVector(const MPCData& d, Formulation& f, QP&)
{
  f.addControlQuadraticVector(d.R_big.diagonal(), d.ctrls_ref);
}

// ---- InputRateCost ----------------------------------------------------------

InputRateCost::InputRateCost(const VectorParameterization& input_param) :
    input_param_{input_param},
    P_c_{MatrixXd::Zero(input_param.numCoefficients(),
                        input_param.numCoefficients())},
    q_c_{VectorXd::Zero(input_param.numCoefficients())}
{
  // Construction only: the dense basis Phi (one row per sample), the
  // differences D Phi, and G = (D Phi)^T (D Phi)
  const Parameterization& param{input_param.parameterization()};
  const int T{param.horizon_steps}, ncp{param.num_control_points};
  MatrixXd Phi{MatrixXd::Zero(T, ncp)};
  for (int k{0}; k < T; ++k)
    for (int i{0}; i < param.band_width; ++i)
      Phi(k, param.band_start(k) + i) = param.band_weights(i, k);
  MatrixXd DPhi{Phi};
  DPhi.bottomRows(T - 1) -= Phi.topRows(T - 1);
  const MatrixXd G{DPhi.transpose() * DPhi};

  for (int j{0}; j < ncp; ++j)
    for (int i{0}; i < ncp; ++i)
      if (G(i, j) != 0.0)
        G_.push_back({i, j, G(i, j)});
  for (int i{0}; i < ncp; ++i)
    if (Phi(0, i) != 0.0)
      phi0_.push_back({i, Phi(0, i)});
}

void InputRateCost::updateMatrix(const MPCData& d, Formulation& f, QP&)
{
  // G times each channel's weight (P_c_ stays zero outside these entries)
  for (int ch{0}; ch < input_param_.inputDim(); ++ch)
    for (const GEntry& e : G_)
      P_c_(input_param_.coefficientIndex(ch, e.i),
           input_param_.coefficientIndex(ch, e.j)) = e.value * d.R_rate(ch);
  f.addControlMatrix(P_c_);
}

void InputRateCost::updateVector(const MPCData& d, Formulation& f, QP&)
{
  // the u_0 - u_prev difference: -Phi_0^T R_rate u_prev
  for (int ch{0}; ch < input_param_.inputDim(); ++ch)
    for (const Phi0Entry& e : phi0_)
      q_c_(input_param_.coefficientIndex(ch, e.i)) =
          -e.value * d.R_rate(ch) * d.u_prev(ch);
  f.addControlLinear(q_c_);
}

void InputRateCost::writeCostPlaceholders(Formulation& f)
{
  // G on every channel (the cost with unit weights, positive semidefinite),
  // so its entries are in the pattern even where a weight is zero now
  for (int ch{0}; ch < input_param_.inputDim(); ++ch)
    for (const GEntry& e : G_)
      P_c_(input_param_.coefficientIndex(ch, e.i),
           input_param_.coefficientIndex(ch, e.j)) = e.value;
  f.addControlCostPlaceholders(P_c_);
}

// ---- InputBounds ------------------------------------------------------------

InputBounds::InputBounds(const VectorParameterization& input_param,
                         const bool saturate_input_trajectory) :
    input_param_{input_param},
    samples_{boundedSamples(input_param, saturate_input_trajectory)},
    num_bounded_{samples_.empty()
                     ? input_param.parameterization().num_control_points
                     : static_cast<int>(samples_.size())}
{}

void InputBounds::declare(QPLayout& layout, Formulation& /*f*/)
{
  rows_ = layout.addRows(input_param_.inputDim() * num_bounded_);
}

void InputBounds::updateMatrix(const MPCData&, Formulation&, QP& qp)
{
  // The control points come first in every formulation, so these rows do not
  // depend on it
  if (samples_.empty()) {
    // bound the control points directly (much fewer constraints)
    qp.A.middleRows(rows_.offset, rows_.rows).diagonal().setOnes();
    return;
  }
  // bound sampled inputs (each a weighted sum of control points)
  const Parameterization& param{input_param_.parameterization()};
  const int m{input_param_.inputDim()};
  int row{rows_.offset};
  for (const int k : samples_) {
    for (int i{0}; i < param.band_width; ++i) {
      const int col{input_param_.coefficientIndex(0, param.band_start(k) + i)};
      qp.A.block(row, col, m, m)
          .diagonal()
          .setConstant(param.band_weights(i, k));
    }
    row += m;
  }
}

void InputBounds::updateVector(const MPCData& d, Formulation&, QP& qp)
{
  qp.l.segment(rows_.offset, rows_.rows) = d.u_min.replicate(num_bounded_, 1);
  qp.u.segment(rows_.offset, rows_.rows) = d.u_max.replicate(num_bounded_, 1);
}

// ---- InitialSlew ------------------------------------------------------------

void InitialSlew::updateMatrix(const MPCData&, Formulation&, QP& qp)
{
  // the first sampled input u_0
  const Parameterization& param{input_param_.parameterization()};
  const int m{input_param_.inputDim()};
  for (int i{0}; i < param.band_width; ++i) {
    const int col{input_param_.coefficientIndex(0, param.band_start(0) + i)};
    qp.A.block(rows_.offset, col, m, m)
        .diagonal()
        .setConstant(param.band_weights(i, 0));
  }
}

void InitialSlew::updateVector(const MPCData& d, Formulation&, QP& qp)
{
  qp.l.segment(rows_.offset, rows_.rows) = d.u_prev - d.u0_slew;
  qp.u.segment(rows_.offset, rows_.rows) = d.u_prev + d.u0_slew;
}

// ---- ControlPointSlew -------------------------------------------------------

void ControlPointSlew::updateMatrix(const MPCData&, Formulation&, QP& qp)
{
  // c_{i+1} - c_i
  auto rows{qp.A.middleRows(rows_.offset, rows_.rows)};
  rows.diagonal().setConstant(-1.0);
  rows.diagonal(input_param_.inputDim()).setOnes();
}

void ControlPointSlew::updateVector(const MPCData& d, Formulation&, QP& qp)
{
  const int num_differences{input_param_.parameterization().num_control_points
                            - 1};
  qp.u.segment(rows_.offset, rows_.rows) =
      d.ctrls_slew.replicate(num_differences, 1);
  qp.l.segment(rows_.offset, rows_.rows) =
      -qp.u.segment(rows_.offset, rows_.rows);
}

// ---- StateBounds ------------------------------------------------------------

StateBounds::StateBounds(const std::vector<int>& states,
                         const int state_dim,
                         const int horizon_steps,
                         const bool soft) :
    states_{StateRowMap::selection(states, state_dim)},
    T_{horizon_steps},
    soft_{soft},
    minus_inf_{VectorXd::Constant(states_.rows(), -MPCData::inf)},
    plus_inf_{VectorXd::Constant(states_.rows(), MPCData::inf)}
{}

void StateBounds::declare(QPLayout& layout, Formulation& /*f*/)
{
  const int num_bounded{states_.rows()};
  if (!soft_) {
    rows_ = layout.addRows(num_bounded * T_);
    return;
  }
  slack_ = layout.addVariables(num_bounded);
  upper_ = layout.addRows(num_bounded * T_);
  lower_ = layout.addRows(num_bounded * T_);
  nonneg_ = layout.addRows(num_bounded);
  rows_ = {upper_.offset, 2 * num_bounded * T_ + num_bounded};
}

void StateBounds::updateMatrix(const MPCData&, Formulation& f, QP& qp)
{
  if (!soft_) {
    f.setStateRows(rows_, states_, 1, T_);
    return;
  }
  f.setStateRows(upper_, states_, 1, T_);
  f.setStateRows(lower_, states_, 1, T_);
  // the slack columns: -s in the upper rows, +s in the lower rows, s >= 0
  const int num_bounded{states_.rows()};
  for (int k{0}; k < T_; ++k) {
    for (int j{0}; j < num_bounded; ++j) {
      qp.A(upper_.offset + k * num_bounded + j, slack_.offset + j) = -1.0;
      qp.A(lower_.offset + k * num_bounded + j, slack_.offset + j) = 1.0;
    }
  }
  for (int j{0}; j < num_bounded; ++j)
    qp.A(nonneg_.offset + j, slack_.offset + j) = 1.0;
}

void StateBounds::updateVector(const MPCData& d, Formulation& f, QP& qp)
{
  if (!soft_) {
    f.setStateBounds(rows_, states_, 1, T_, d.x_min, d.x_max);
    return;
  }
  f.setStateBounds(upper_, states_, 1, T_, minus_inf_, d.x_max);
  f.setStateBounds(lower_, states_, 1, T_, d.x_min, plus_inf_);
  qp.l.segment(nonneg_.offset, nonneg_.rows).setZero();
  qp.u.segment(nonneg_.offset, nonneg_.rows) = plus_inf_;
}

// ---- GeneralConstraints -----------------------------------------------------

GeneralConstraints::GeneralConstraints(
    const VectorParameterization& input_param,
    const int num_constraints,
    const int state_dim) :
    input_param_{input_param},
    p_{num_constraints},
    T_{input_param.parameterization().horizon_steps},
    Hx_{StateRowMap::general(num_constraints, state_dim)},
    Hx0_{num_constraints}
{}

void GeneralConstraints::declare(QPLayout& layout, Formulation& /*f*/)
{
  rows_ = layout.addRows(p_ * T_);
  state_rows_ = {rows_.offset + p_, p_ * (T_ - 1)};
}

void GeneralConstraints::addInputPart(const MPCData& d,
                                      QP& qp,
                                      const int k,
                                      const int row) const
{
  const Parameterization& param{input_param_.parameterization()};
  const int m{input_param_.inputDim()};
  for (int i{0}; i < param.band_width; ++i) {
    const int col{input_param_.coefficientIndex(0, param.band_start(k) + i)};
    qp.A.block(row, col, p_, m) += param.band_weights(i, k) * d.Hu;
  }
}

void GeneralConstraints::updateMatrix(const MPCData& d, Formulation& f, QP& qp)
{
  Hx_.assign(d.Hx);
  // k = 0: Hu u_0 only (x_0 is not a QP variable in any formulation)
  qp.A.block(rows_.offset, 0, p_, input_param_.numCoefficients()).setZero();
  addInputPart(d, qp, 0, rows_.offset);
  // k >= 1: the formulation writes Hx x_k (clearing the control point columns),
  // then Hu u_k is added
  f.setStateRows(state_rows_, Hx_, 1, T_ - 1);
  for (int k{1}; k < T_; ++k)
    addInputPart(d, qp, k, rows_.offset + k * p_);
}

void GeneralConstraints::updateVector(const MPCData& d, Formulation& f, QP& qp)
{
  // Hx_ was refreshed by updateMatrix(), which runs first on every change of
  // Hx (ConstraintMatrices is in both parts' dependencies)
  assert(Hx_.matches(d.Hx));
  // k = 0: h - Hx x_0
  Hx0_.noalias() = d.Hx * d.x0;
  qp.l.segment(rows_.offset, p_) = d.h_min - Hx0_;
  qp.u.segment(rows_.offset, p_) = d.h_max - Hx0_;
  f.setStateBounds(state_rows_, Hx_, 1, T_ - 1, d.h_min, d.h_max);
}

// ---- TerminalConstraints ----------------------------------------------------

void TerminalConstraints::updateMatrix(const MPCData& d, Formulation& f, QP&)
{
  Hf_.assign(d.Hf);
  f.setStateRows(rows_, Hf_, T_, 1);
}

void TerminalConstraints::updateVector(const MPCData& d, Formulation& f, QP&)
{
  assert(Hf_.matches(
      d.Hf)); // refreshed by updateMatrix(), as for the general rows
  f.setStateBounds(rows_, Hf_, T_, 1, d.hf_min, d.hf_max);
}

// ---- SoftStatePenalty -------------------------------------------------------

void SoftStatePenalty::updateMatrix(const MPCData& d, Formulation& f, QP&)
{
  // 1/2 of s^T diag(l2) s
  f.addVariableQuadratic(slack_, d.x_soft_l2);
}

void SoftStatePenalty::updateVector(const MPCData& d, Formulation& f, QP&)
{
  // 1/2 of l1^T s
  half_l1_.noalias() = 0.5 * d.x_soft_l1;
  f.addVariableLinear(slack_, half_l1_);
}

} // namespace affine_mpc::detail
