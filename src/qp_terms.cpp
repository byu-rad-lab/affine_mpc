#include "qp_terms.hpp"

#include <Eigen/Core>
#include <cassert>
#include <cmath>
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

// ---- ChannelMatrix ----------------------------------------------------------

ChannelMatrix::ChannelMatrix(const MatrixXd& G)
{
  for (Index j{0}; j < G.cols(); ++j)
    for (Index i{0}; i < G.rows(); ++i)
      if (G(i, j) != 0.0)
        G_.push_back({static_cast<int>(i), static_cast<int>(j), G(i, j)});
}

void ChannelMatrix::declare(const VectorParameterization& input_param,
                            QPLayout& layout,
                            FormulationImpl& f)
{
  const int m{input_param.inputDim()};
  const Index size{m * static_cast<Index>(G_.size())};
  P_c_.reserve(static_cast<std::size_t>(size));
  P_c_values_.resize(size);
  for (int ch{0}; ch < m; ++ch)
    for (const Entry& e : G_)
      P_c_.declare(input_param.coefficientIndex(ch, e.i),
                   input_param.coefficientIndex(ch, e.j));
  f.declareControlMatrix(layout, P_c_);
}

void ChannelMatrix::add(const VectorParameterization& input_param,
                        const VectorXd& weights,
                        FormulationImpl& f)
{
  // G times each channel's weight
  Index idx{0};
  for (int ch{0}; ch < input_param.inputDim(); ++ch)
    for (const Entry& e : G_)
      P_c_values_(idx++) = e.value * weights(ch);
  f.addControlMatrix(P_c_, P_c_values_);
}

MatrixXd sampledBasis(const Parameterization& param)
{
  MatrixXd Phi{MatrixXd::Zero(param.horizon_steps, param.num_control_points)};
  for (int k{0}; k < param.horizon_steps; ++k)
    for (int i{0}; i < param.band_width; ++i)
      Phi(k, param.band_start(k) + i) = param.band_weights(i, k);
  return Phi;
}

// ---- InputTrajectoryCost ----------------------------------------------------

InputTrajectoryCost::InputTrajectoryCost(
    const VectorParameterization& input_param) :
    input_param_{input_param},
    G_{[&input_param] {
      // Construction only: G = Phi^T Phi
      const MatrixXd Phi{sampledBasis(input_param.parameterization())};
      return MatrixXd{Phi.transpose() * Phi};
    }()},
    q_c_{VectorXd::Zero(input_param.numCoefficients())}
{}

void InputTrajectoryCost::declare(QPLayout& layout, FormulationImpl& f)
{
  G_.declare(input_param_, layout, f);
}

void InputTrajectoryCost::updateMatrix(const MPCData& d,
                                       FormulationImpl& f,
                                       QP&)
{
  G_.add(input_param_, d.R, f);
}

void InputTrajectoryCost::updateVector(const MPCData& d,
                                       FormulationImpl& f,
                                       QP&)
{
  // -Phi^T R u_ref, channel by channel, from the basis band of each sample
  const Parameterization& param{input_param_.parameterization()};
  const int m{input_param_.inputDim()};
  q_c_.setZero();
  for (int k{0}; k < param.horizon_steps; ++k) {
    for (int i{0}; i < param.band_width; ++i) {
      const double w{param.band_weights(i, k)};
      const int cp{param.band_start(k) + i};
      for (int ch{0}; ch < m; ++ch)
        q_c_(input_param_.coefficientIndex(ch, cp)) -=
            w * d.R(ch) * d.u_ref(k * m + ch);
    }
  }
  f.addControlLinear(q_c_);
}

// ---- InputRateCost ----------------------------------------------------------

InputRateCost::InputRateCost(const VectorParameterization& input_param) :
    InputRateCost{input_param, sampledBasis(input_param.parameterization())}
{}

InputRateCost::InputRateCost(const VectorParameterization& input_param,
                             const MatrixXd& Phi) :
    input_param_{input_param},
    G_{[&Phi] {
      // Construction only: the differences D Phi, and G = (D Phi)^T (D Phi)
      const Index T{Phi.rows()};
      MatrixXd DPhi{Phi};
      DPhi.bottomRows(T - 1) -= Phi.topRows(T - 1);
      return MatrixXd{DPhi.transpose() * DPhi};
    }()},
    q_c_{VectorXd::Zero(input_param.numCoefficients())}
{
  for (Index i{0}; i < Phi.cols(); ++i)
    if (Phi(0, i) != 0.0)
      phi0_.push_back({static_cast<int>(i), Phi(0, i)});
}

void InputRateCost::declare(QPLayout& layout, FormulationImpl& f)
{
  G_.declare(input_param_, layout, f);
}

void InputRateCost::updateMatrix(const MPCData& d, FormulationImpl& f, QP&)
{
  G_.add(input_param_, d.R_rate, f);
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
// property), and so does bounding the control points of a refined
// representation of the same input curve (after knot insertion). A custom
// basis has no such property, so its sampled inputs are always bounded.
InputBounds::BoundRows
InputBounds::boundRows(const VectorParameterization& input_param,
                       const InputBoundMode mode,
                       const int min_refined_intervals)
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
          ? input_param.refineKnots(min_refined_intervals)
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
                         const int min_refined_intervals) :
    input_param_{input_param},
    M_{boundRows(input_param, mode, min_refined_intervals)},
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

// ---- SlewBounds -------------------------------------------------------------

namespace {

// Whether the control point differences bound the sampled changes exactly:
// always for degree 0 (each change is zero or one difference), and for degree
// 1 when every knot interval [t_i, t_{i+1}] contains a whole sample step
// [k-1, k], over which the change equals the interval's slope (a step across
// knots changes by a convex combination of the slopes it spans, so it adds no
// limit). A zero-length interval (a jump) does not, since its factor 0 would
// forbid the jump.
bool controlPointSlewIsExact(const Parameterization& param)
{
  if (param.type != Parameterization::Type::BSpline || param.degree > 1)
    return false;
  if (param.degree == 0)
    return true;
  constexpr double snap_tol{1e-9}; // round-off from integer knots
  for (int i{1}; i < param.num_control_points; ++i) {
    const double first{std::ceil(param.knots(i) - snap_tol)};
    const double last{std::floor(param.knots(i + 1) + snap_tol)};
    if (first + 1.0 > last)
      return false;
  }
  return true;
}

} // namespace

SlewBounds::SlewBounds(const VectorParameterization& input_param,
                       const SlewBoundMode mode) :
    input_param_{input_param}
{
  const Parameterization& param{input_param.parameterization()};
  const bool control_point_rows{param.type == Parameterization::Type::BSpline
                                && (mode == SlewBoundMode::ControlPoints
                                    || controlPointSlewIsExact(param))};
  if (control_point_rows) {
    // c_i - c_{i-1}. The derivative of a degree-p B-spline has the control
    // points p (c_i - c_{i-1}) / (t_{i+p} - t_i) (knots in samples), so by
    // the convex hull property and the mean value theorem, bounding them by
    // du_max bounds every sampled change. Degree 0 changes by exactly the
    // differences.
    const int p{param.degree};
    const auto num_rows{static_cast<std::size_t>(param.num_control_points - 1)};
    D_.reserve(2 * num_rows);
    factors_.reserve(num_rows);
    for (int i{1}; i < param.num_control_points; ++i) {
      D_.push_back({i - 1, i - 1, -1.0});
      D_.push_back({i - 1, i, 1.0});
      factors_.push_back(p == 0 ? 1.0
                                : (param.knots(i + p) - param.knots(i)) / p);
    }
    return;
  }

  // Construction only: the sampled differences D Phi, k = 1, ..., T-1,
  // omitting rows that are negligible next to the largest (e.g. where a custom
  // basis has decayed), as the input bounds of a custom basis do
  const MatrixXd Phi{sampledBasis(param)};
  const Index num_steps{Phi.rows() - 1};
  if (num_steps <= 0)
    return;
  const MatrixXd DPhi{Phi.bottomRows(num_steps) - Phi.topRows(num_steps)};
  const VectorXd norms{DPhi.rowwise().norm()};
  constexpr double prune_tol{1e-9};
  const double keep_above{prune_tol * norms.maxCoeff()};
  int row{0};
  for (Index k{0}; k < num_steps; ++k) {
    if (!(norms(k) > keep_above))
      continue;
    for (Index i{0}; i < DPhi.cols(); ++i)
      if (DPhi(k, i) != 0.0)
        D_.push_back({row, static_cast<int>(i), DPhi(k, i)});
    factors_.push_back(1.0);
    ++row;
  }
}

void SlewBounds::declare(QPLayout& layout, FormulationImpl& /*f*/)
{
  const int m{input_param_.inputDim()};
  rows_ = layout.addRows(m * static_cast<int>(factors_.size()));
  // The control points come first in every formulation, so these rows do not
  // depend on it
  for (const Entry& e : D_)
    for (int ch{0}; ch < m; ++ch)
      A_.declare(rows_.offset + e.row * m + ch,
                 input_param_.coefficientIndex(ch, e.cp), e.value);
  layout.declareA(A_.entries);
}

void SlewBounds::updateMatrix(const MPCData&, FormulationImpl&, QP&)
{
  A_.write();
}

void SlewBounds::updateVector(const MPCData& d, FormulationImpl&, QP& qp)
{
  const int m{input_param_.inputDim()};
  int row{rows_.offset};
  for (const double factor : factors_) {
    for (int ch{0}; ch < m; ++ch, ++row) {
      // an infinite limit stays infinite on a factor-0 row (0 * inf is NaN)
      const double du{d.slew_rate(ch)};
      const double bound{std::isinf(du) ? du : factor * du};
      qp.l(row) = -bound;
      qp.u(row) = bound;
    }
  }
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
