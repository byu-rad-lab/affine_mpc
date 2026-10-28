#include "qp_terms.hpp"

#include <Eigen/Core>
#include <cassert>
#include <cstddef>
#include <vector>

#include "affine_mpc/parameterization.hpp"
#include "formulation.hpp"

using namespace Eigen;

namespace affine_mpc::detail {

// ---- StateCost --------------------------------------------------------------

void StateCost::updateMatrix(const MPCData& d, FormulationImpl& f, QP&)
{
  f.addStateQuadraticMatrix(d.Q_big.diagonal());
}

void StateCost::updateVector(const MPCData& d, FormulationImpl& f, QP&)
{
  f.addStateQuadraticVector(d.Q_big.diagonal(), d.x_ref);
}

// ---- InputCost --------------------------------------------------------------

void InputCost::updateMatrix(const MPCData& d, FormulationImpl& f, QP&)
{
  f.addControlQuadraticMatrix(d.R_big.diagonal());
}

void InputCost::updateVector(const MPCData& d, FormulationImpl& f, QP&)
{
  f.addControlQuadraticVector(d.R_big.diagonal(), d.ctrls_ref);
}

// ---- InputRateCost ----------------------------------------------------------

InputRateCost::InputRateCost(const VectorParameterization& input_param) :
    input_param_{input_param},
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

void InputRateCost::declare(QPLayout& layout, FormulationImpl& f)
{
  const int m{input_param_.inputDim()};
  const Index size{m * static_cast<Index>(G_.size())};
  P_c_.reserve(static_cast<std::size_t>(size));
  P_c_values_.resize(size);
  for (int ch{0}; ch < m; ++ch)
    for (const GEntry& e : G_)
      P_c_.declare(input_param_.coefficientIndex(ch, e.i),
                   input_param_.coefficientIndex(ch, e.j));
  f.declareControlMatrix(layout, P_c_);
}

void InputRateCost::updateMatrix(const MPCData& d, FormulationImpl& f, QP&)
{
  // G times each channel's weight
  Index idx{0};
  for (int ch{0}; ch < input_param_.inputDim(); ++ch)
    for (const GEntry& e : G_)
      P_c_values_(idx++) = e.value * d.R_rate(ch);
  f.addControlMatrix(P_c_, P_c_values_);
}

void InputRateCost::updateVector(const MPCData& d, FormulationImpl& f, QP&)
{
  // the u_0 - u_prev difference: -Phi_0^T R_rate u_prev
  for (int ch{0}; ch < input_param_.inputDim(); ++ch)
    for (const Phi0Entry& e : phi0_)
      q_c_(input_param_.coefficientIndex(ch, e.i)) =
          -e.value * d.R_rate(ch) * d.u_prev(ch);
  f.addControlLinear(q_c_);
}

// ---- InputBounds ------------------------------------------------------------

// Bounding B-spline control points bounds the sampled inputs (convex hull
// property), and so does bounding the control points of the same spline after
// knot insertion. A custom basis has no such property, so its sampled inputs
// are always bounded.
InputBounds::BoundRows
InputBounds::boundRows(const VectorParameterization& input_param,
                       const InputBoundMode mode,
                       const int refined_bound_interval)
{
  const Parameterization& param{input_param.parameterization()};
  const bool bspline{param.type == Parameterization::Type::BSpline};
  BoundRows M;
  if (bspline && mode == InputBoundMode::ControlPoints)
    return M;

  // Refined, degree >= 2: the refined control points the samples depend on,
  // unless they are at least as many as the samples (T); the sampled rows are
  // then exact and fewer. Degree 0 and 1 already have exact bounds with few
  // rows, so Refined uses them (the control points for degree 0, the knot
  // samples for degree 1).
  const VectorParameterization::KnotRefinement r{
      bspline && mode == InputBoundMode::Refined && param.degree >= 2
          ? input_param.refineKnots(refined_bound_interval)
          : VectorParameterization::KnotRefinement{}};
  if (!r.sampled.empty()
      && r.sampled.size() < std::size_t(param.horizon_steps)) {
    const int num_rows{static_cast<int>(r.sampled.size())};
    M.start.reserve(r.sampled.size());
    M.weights.resize(r.weights.rows(), num_rows);
    for (int row{0}; row < num_rows; ++row) {
      const int j{r.sampled[static_cast<std::size_t>(row)]};
      M.start.push_back(r.start(j));
      M.weights.col(row) = r.weights.col(j);
    }
    M.skip_zeros = true;
    return M;
  }

  // the sampled inputs that make the bound exact (empty for degree 0: the
  // control points are the sampled inputs)
  const std::vector<int> samples{input_param.inputSaturationSamples()};
  const int num_rows{static_cast<int>(samples.size())};
  M.start.reserve(samples.size());
  M.weights.resize(param.band_width, num_rows);
  for (int row{0}; row < num_rows; ++row) {
    const int k{samples[static_cast<std::size_t>(row)]};
    M.start.push_back(param.band_start(k));
    M.weights.col(row) = param.band_weights.col(k);
  }
  return M;
}

InputBounds::InputBounds(const VectorParameterization& input_param,
                         const InputBoundMode mode,
                         const int refined_bound_interval) :
    input_param_{input_param},
    M_{boundRows(input_param, mode, refined_bound_interval)},
    num_bounded_{M_.start.empty()
                     ? input_param.parameterization().num_control_points
                     : static_cast<int>(M_.start.size())}
{}

void InputBounds::declare(QPLayout& layout, FormulationImpl& /*f*/)
{
  rows_ = layout.addRows(input_param_.inputDim() * num_bounded_);
  // The control points come first in every formulation, so these rows do not
  // depend on it
  if (M_.start.empty()) {
    // bound the control points directly (much fewer constraints)
    for (int i{0}; i < rows_.rows; ++i)
      A_.declare(rows_.offset + i, i, 1.0);
  } else {
    // bound weighted sums of control points (sampled inputs or refined
    // control points), with the same weights for every channel
    const int m{input_param_.inputDim()};
    int row{rows_.offset};
    for (int r{0}; r < num_bounded_; ++r) {
      for (int i{0}; i < M_.weights.rows(); ++i) {
        const double w{M_.weights(i, r)};
        if (M_.skip_zeros && w == 0.0)
          continue;
        const int col{input_param_.coefficientIndex(
            0, M_.start[static_cast<std::size_t>(r)] + i)};
        for (int ch{0}; ch < m; ++ch)
          A_.declare(row + ch, col + ch, w);
      }
      row += m;
    }
  }
  layout.declareA(A_.entries);
}

void InputBounds::updateMatrix(const MPCData&, FormulationImpl&, QP&)
{
  A_.write();
}

void InputBounds::updateVector(const MPCData& d, FormulationImpl&, QP& qp)
{
  qp.l.segment(rows_.offset, rows_.rows) = d.u_min.replicate(num_bounded_, 1);
  qp.u.segment(rows_.offset, rows_.rows) = d.u_max.replicate(num_bounded_, 1);
}

// ---- InitialSlew ------------------------------------------------------------

void InitialSlew::declare(QPLayout& layout, FormulationImpl& /*f*/)
{
  rows_ = layout.addRows(input_param_.inputDim());
  // the first sampled input u_0
  const Parameterization& param{input_param_.parameterization()};
  const int m{input_param_.inputDim()};
  for (int i{0}; i < param.band_width; ++i) {
    const int col{input_param_.coefficientIndex(0, param.band_start(0) + i)};
    for (int ch{0}; ch < m; ++ch)
      A_.declare(rows_.offset + ch, col + ch, param.band_weights(i, 0));
  }
  layout.declareA(A_.entries);
}

void InitialSlew::updateMatrix(const MPCData&, FormulationImpl&, QP&)
{
  A_.write();
}

void InitialSlew::updateVector(const MPCData& d, FormulationImpl&, QP& qp)
{
  qp.l.segment(rows_.offset, rows_.rows) = d.u_prev - d.u0_slew;
  qp.u.segment(rows_.offset, rows_.rows) = d.u_prev + d.u0_slew;
}

// ---- ControlPointSlew -------------------------------------------------------

void ControlPointSlew::declare(QPLayout& layout, FormulationImpl& /*f*/)
{
  const int m{input_param_.inputDim()};
  rows_ = layout.addRows(input_param_.numCoefficients() - m);
  // c_{i+1} - c_i
  for (int i{0}; i < rows_.rows; ++i)
    A_.declare(rows_.offset + i, i, -1.0);
  for (int i{0}; i < rows_.rows; ++i)
    A_.declare(rows_.offset + i, i + m, 1.0);
  layout.declareA(A_.entries);
}

void ControlPointSlew::updateMatrix(const MPCData&, FormulationImpl&, QP&)
{
  A_.write();
}

void ControlPointSlew::updateVector(const MPCData& d, FormulationImpl&, QP& qp)
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

void StateBounds::declare(QPLayout& layout, FormulationImpl& f)
{
  const int num_bounded{states_.rows()};
  if (!soft_) {
    rows_ = layout.addRows(num_bounded * T_);
    hard_rows_ = f.declareStateRows(layout, rows_, states_, 1, T_);
    return;
  }
  slack_ = layout.addVariables(num_bounded);
  upper_ = layout.addRows(num_bounded * T_);
  lower_ = layout.addRows(num_bounded * T_);
  nonneg_ = layout.addRows(num_bounded);
  rows_ = {upper_.offset, 2 * num_bounded * T_ + num_bounded};
  upper_rows_ = f.declareStateRows(layout, upper_, states_, 1, T_);
  lower_rows_ = f.declareStateRows(layout, lower_, states_, 1, T_);
  // the slack columns: -s in the upper rows, +s in the lower rows, s >= 0
  for (int k{0}; k < T_; ++k) {
    for (int j{0}; j < num_bounded; ++j) {
      slack_A_.declare(upper_.offset + k * num_bounded + j, slack_.offset + j,
                       -1.0);
      slack_A_.declare(lower_.offset + k * num_bounded + j, slack_.offset + j,
                       1.0);
    }
  }
  for (int j{0}; j < num_bounded; ++j)
    slack_A_.declare(nonneg_.offset + j, slack_.offset + j, 1.0);
  layout.declareA(slack_A_.entries);
}

void StateBounds::updateMatrix(const MPCData&, FormulationImpl& f, QP&)
{
  if (!soft_) {
    f.setStateRows(hard_rows_, states_);
    return;
  }
  f.setStateRows(upper_rows_, states_);
  f.setStateRows(lower_rows_, states_);
  slack_A_.write();
}

void StateBounds::updateVector(const MPCData& d, FormulationImpl& f, QP& qp)
{
  if (!soft_) {
    f.setStateBounds(hard_rows_, states_, d.x_min, d.x_max);
    return;
  }
  f.setStateBounds(upper_rows_, states_, minus_inf_, d.x_max);
  f.setStateBounds(lower_rows_, states_, d.x_min, plus_inf_);
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

void GeneralConstraints::declare(QPLayout& layout, FormulationImpl& f)
{
  rows_ = layout.addRows(p_ * T_);
  state_rows_ = f.declareStateRows(layout, {rows_.offset + p_, p_ * (T_ - 1)},
                                   Hx_, 1, T_ - 1);
  const Parameterization& param{input_param_.parameterization()};
  const int m{input_param_.inputDim()};
  Hu_.reserve(static_cast<std::size_t>(T_ * param.band_width * p_ * m));
  for (int k{0}; k < T_; ++k) {
    for (int i{0}; i < param.band_width; ++i) {
      const int col{input_param_.coefficientIndex(0, param.band_start(k) + i)};
      Hu_.declareBlock(rows_.offset + k * p_, col, p_, m);
    }
  }
  layout.declareA(Hu_);
}

void GeneralConstraints::updateMatrix(const MPCData& d, FormulationImpl& f, QP&)
{
  Hx_.assign(d.Hx);
  // k = 0: Hu u_0 only (x_0 is not a QP variable in any formulation). k >= 1:
  // Hx x_k by the formulation, which may assign over the zeroed Hu entries,
  // then Hu u_k added.
  Hu_.setZero();
  f.setStateRows(state_rows_, Hx_);
  const Parameterization& param{input_param_.parameterization()};
  const Index block{d.Hu.size()};
  Index first{0};
  for (int k{0}; k < T_; ++k) {
    for (int i{0}; i < param.band_width; ++i, first += block)
      Hu_.addBlock(first, param.band_weights(i, k), d.Hu);
  }
}

void GeneralConstraints::updateVector(const MPCData& d,
                                      FormulationImpl& f,
                                      QP& qp)
{
  // Hx_ was refreshed by updateMatrix(), which runs first on every change of
  // Hx (ConstraintMatrices is in both parts' dependencies)
  assert(Hx_.matches(d.Hx));
  // k = 0: h - Hx x_0
  Hx0_.noalias() = d.Hx * d.x0;
  qp.l.segment(rows_.offset, p_) = d.h_min - Hx0_;
  qp.u.segment(rows_.offset, p_) = d.h_max - Hx0_;
  f.setStateBounds(state_rows_, Hx_, d.h_min, d.h_max);
}

// ---- TerminalConstraints ----------------------------------------------------

void TerminalConstraints::updateMatrix(const MPCData& d,
                                       FormulationImpl& f,
                                       QP&)
{
  Hf_.assign(d.Hf);
  f.setStateRows(state_rows_, Hf_);
}

void TerminalConstraints::updateVector(const MPCData& d,
                                       FormulationImpl& f,
                                       QP&)
{
  assert(Hf_.matches(
      d.Hf)); // refreshed by updateMatrix(), as for the general rows
  f.setStateBounds(state_rows_, Hf_, d.hf_min, d.hf_max);
}

// ---- SoftStatePenalty -------------------------------------------------------

void SoftStatePenalty::updateMatrix(const MPCData& d, FormulationImpl& f, QP&)
{
  // 1/2 of s^T diag(l2) s
  f.addVariableQuadratic(slack_, d.x_soft_l2);
}

void SoftStatePenalty::updateVector(const MPCData& d, FormulationImpl& f, QP&)
{
  // 1/2 of l1^T s
  half_l1_.noalias() = 0.5 * d.x_soft_l1;
  f.addVariableLinear(slack_, half_l1_);
}

} // namespace affine_mpc::detail
