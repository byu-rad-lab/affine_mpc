#include "affine_mpc/parameterization.hpp"

#include <Eigen/Core>
#include <Eigen/SVD>
#include <algorithm>
#include <cassert>
#include <cmath>
#include <iomanip>
#include <numeric>
#include <ostream>
#include <sstream>
#include <stdexcept>
#include <unsupported/Eigen/Splines>

#include "eigen_compat.hpp" // remove this once Eigen 3.5 is required

namespace affine_mpc {

using namespace Eigen;
// revert back to this once Eigen 3.5 is required
// namespace ph = Eigen::placeholders;

namespace { // utilities for this file

void validateHorizonSteps(int horizon_steps)
{
  if (horizon_steps < 1)
    throw std::invalid_argument("[Parameterization::validateHorizonSteps] "
                                "horizon_steps must be positive.\n");
}

void validateDegree(int degree, int horizon_steps)
{
  static const std::string prefix{"[Parameterization::validateDegree] "};
  if (degree < 0)
    throw std::invalid_argument(prefix + "degree can not be negative.\n");
  if (degree >= horizon_steps)
    throw std::invalid_argument(prefix
                                + "degree must be less than horizon_steps.\n");
}

void validateNumControls(int num_control_points, int horizon_steps, int degree)
{
  static const std::string prefix{"[Parameterization::validateNumControls] "};
  if (num_control_points < degree + 1)
    throw std::invalid_argument(
        prefix + "must have at least degree+1 control points.\n");
  if (num_control_points > horizon_steps)
    throw std::invalid_argument(
        prefix + "num_control_points can not be greater than horizon_steps.\n");
}

void validateKnots(const Ref<const VectorXd>& knots,
                   int horizon_steps,
                   int degree)
{
  static const std::string prefix{"[Parameterization::validateKnots] "};
  const int num_knots = knots.size();
  if (num_knots < 2 * (degree + 1))
    throw std::invalid_argument(
        prefix + "Size of knots must be at least 2*(degree+1).\n");
  const int num_ctrls{num_knots - degree - 1};
  if (num_ctrls > horizon_steps)
    throw std::invalid_argument(
        prefix + "Size of knots can not exceed horizon_steps+degree+1.\n");

  if (knots(degree) != 0.0) {
    std::stringstream ss;
    ss << std::setprecision(17)
       << "[Parameterization::validateKnots] First active knot must equal zero "
       << "(representing input at current time step, k=0).\n    "
       << "Got " << knots(degree) << std::endl;
    throw std::invalid_argument(ss.str());
  }
  if (knots(ph::last - degree) != horizon_steps - 1) {
    std::stringstream ss;
    ss << std::setprecision(17) << "[Parameterization::validateKnots] "
       << "Last active knot must equal horizon_steps-1 "
       << "(representing final input).\n    Got "
       << knots(knots.size() - degree - 1)
       << " with horizon_steps = " << horizon_steps << std::endl;
    throw std::invalid_argument(ss.str());
  }
  const VectorXd diff{knots(seq(1, ph::last)) - knots(seq(0, ph::last - 1))};
  if (diff.minCoeff() < 0)
    throw std::invalid_argument(
        "[Parameterization::validateKnots] knots must be non-decreasing.\n");

  // Repeated knots are allowed (they lower the continuity at that knot) as
  // long as the sampled basis keeps full column rank, which is checked once
  // the basis is computed. Two cases are rejected here instead, because the
  // span search would pick an empty knot interval (Eigen then divides by zero,
  // and versions differ in which interval they pick at the first knot):
  // - an interior knot equal to the first active knot 0 (any degree): the
  //   first basis function is zero from 0 on (the spline is evaluated from the
  //   right);
  // - for degree > 0, an interior knot equal to the last active knot T-1: the
  //   last basis function is zero up to T-1 (the spline is evaluated there
  //   from the left). Degree 0 allows it: the last control point owns the last
  //   sample, so a change point at T-1 is a one-sample block.
  // Without interior knots (e.g. horizon_steps = 1, where the first and last
  // active knots are both 0) neither case applies.
  const int first_active{degree}, last_active{num_knots - degree - 1};
  const bool has_interior{last_active - first_active > 1};
  const bool first_repeated{has_interior
                            && knots(first_active + 1) == knots(first_active)};
  const bool last_repeated{has_interior && degree > 0
                           && knots(last_active - 1) == knots(last_active)};
  if (first_repeated || last_repeated) {
    const int ctrl{first_repeated ? 0 : last_active - 1};
    std::stringstream ss;
    ss << std::setprecision(10) << "[Parameterization::validateBasisRank] "
       << "the sampled basis does not have full column rank: control point "
       << ctrl << " has no sample of its own. Its basis function, with "
       << "support [" << knots(ctrl) << ", " << knots(ctrl + degree + 1)
       << "], is zero at every sample because an interior knot equals the "
       << (first_repeated ? "first active knot 0 (the spline is evaluated "
                            "there from the right). Move that knot above 0.\n"
                          : "last active knot horizon_steps-1 (the spline is "
                            "evaluated there from the left). Move that knot "
                            "below horizon_steps-1.\n");
    throw std::invalid_argument(ss.str());
  }
}

// Index of the first control point of the band at each sample
VectorXi computeBandStart(int horizon_steps, int degree, const VectorXd& knots)
{
  using Spline1d = Spline<double, 1>;
  VectorXi start{horizon_steps};
  for (int k{0}; k < horizon_steps; ++k) {
    const double t = k;
    start(k) = static_cast<int>(Spline1d::Span(t, degree, knots)) - degree;
  }
  return start;
}

// Nonzero basis function values at each sample (one column per sample)
MatrixXd
computeBandWeights(int horizon_steps, int degree, const VectorXd& knots)
{
  using Spline1d = Spline<double, 1>;
  MatrixXd weights{degree + 1, horizon_steps};
  for (int k{0}; k < horizon_steps; ++k) {
    const double t = k;
    weights.col(k) =
        Spline1d::BasisFunctions(t, degree, knots).matrix().transpose();
  }
  return weights;
}

bool basisNonzero(const VectorXi& band_start,
                  const MatrixXd& band_weights,
                  int control_point,
                  int k)
{
  const Index i{control_point - band_start(k)};
  // a non-finite weight (not expected) does not count as nonzero
  return i >= 0 && i < band_weights.rows() && band_weights(i, k) != 0.0
         && std::isfinite(band_weights(i, k));
}

// Describes the samples at which a basis function is nonzero (contiguous for
// B-splines), e.g. "no sample", "sample k = 4 only", or "samples k = 1, ..., 3"
std::string nonzeroSamples(const VectorXi& band_start,
                           const MatrixXd& band_weights,
                           int control_point)
{
  int first{-1}, last{-1};
  for (int k{0}; k < band_start.size(); ++k) {
    if (basisNonzero(band_start, band_weights, control_point, k)) {
      if (first < 0)
        first = k;
      last = k;
    }
  }
  std::stringstream ss;
  if (first < 0)
    ss << "no sample";
  else if (first == last)
    ss << "sample k = " << first << " only";
  else if (last == first + 1)
    ss << "samples k = " << first << ", " << last;
  else
    ss << "samples k = " << first << ", ..., " << last;
  return ss.str();
}

// Schoenberg-Whitney condition on the sampled basis. The T x eta matrix of
// basis functions sampled at k = 0, ..., T-1 has full column rank if and only
// if the control points can be matched to strictly increasing samples
// k_0 < k_1 < ... < k_{eta-1} with basis function j nonzero at k_j. The
// supports of consecutive basis functions move monotonically to the right, so
// matching each control point to the first free sample at which it is nonzero
// decides it in O(T + eta). Without full rank, some combination of control
// points has no effect on any sampled input and the QP is flat in that
// direction. Returns an empty string when the condition holds, otherwise a
// description of the first control point without a sample.
std::string rankDeficiency(int degree,
                           const VectorXd& knots,
                           const VectorXi& band_start,
                           const MatrixXd& band_weights)
{
  const int T{static_cast<int>(band_start.size())};
  const int num_ctrls{static_cast<int>(knots.size()) - degree - 1};
  int k{0}, prev{-1};
  for (int j{0}; j < num_ctrls; ++j) {
    while (k < T && !basisNonzero(band_start, band_weights, j, k)) {
      if (band_start(k) > j) { // later bands start even further right
        k = T;
        break;
      }
      ++k;
    }
    if (k < T) {
      prev = k++;
      continue;
    }

    std::stringstream ss;
    ss << std::setprecision(10)
       << "the sampled basis does not have full column rank: control point "
       << j << " has no sample of its own. Its basis function, with support ["
       << knots(j) << ", " << knots(j + degree + 1) << "], is nonzero at "
       << nonzeroSamples(band_start, band_weights, j);
    if (j == 1)
      ss << ", but it needs a sample k > " << prev << " (control point 0 "
         << "needs k = " << prev << ")";
    else if (j > 1)
      ss << ", but it needs a sample k > " << prev << " (control points 0 to "
         << j - 1 << " need samples up to k = " << prev << ")";
    ss << ". Each control point needs its own sample at which its basis "
       << "function is nonzero, in increasing order (Schoenberg-Whitney "
       << "condition); otherwise some combination of control points has no "
       << "effect on any sampled input.";
    return ss.str();
  }
  return {};
}

void validateBasisRank(const Parameterization& p)
{
  // not expected after validateKnots; a NaN would count as nonzero below
  if (!p.band_weights.allFinite())
    throw std::invalid_argument(
        "[Parameterization::validateBasisRank] evaluating the basis gave "
        "non-finite values; check the knots.\n");
  const std::string reason{
      rankDeficiency(p.degree, p.knots, p.band_start, p.band_weights)};
  if (!reason.empty())
    throw std::invalid_argument("[Parameterization::validateBasisRank] "
                                + reason
                                + " Spread the knots or lower their "
                                  "multiplicity.\n");
}

// Validates the knots after snapping the boundary knots (the first and last
// degree+1, which include the first and last active knots) to exactly 0 and
// horizon_steps-1 when they are within round-off, e.g. from knots computed by
// scaling fractions of the horizon.
VectorXd
validatedKnots(const Ref<const VectorXd>& knots, int horizon_steps, int degree)
{
  validateHorizonSteps(horizon_steps);
  validateDegree(degree, horizon_steps);

  VectorXd snapped{knots};
  const Index num_knots{snapped.size()};
  if (num_knots >= 2 * (degree + 1)) {
    const double tol{1e-9 * horizon_steps};
    const double last{horizon_steps - 1.0};
    for (Index i{0}; i <= degree; ++i) {
      if (std::abs(snapped(i)) <= tol)
        snapped(i) = 0.0;
      if (std::abs(snapped(num_knots - 1 - i) - last) <= tol)
        snapped(num_knots - 1 - i) = last;
    }
  }
  validateKnots(snapped, horizon_steps, degree);
  return snapped;
}

// Clamped knot vector whose active knot intervals have lengths proportional to
// 1, knot_ratio, knot_ratio^2, ..., scaled to span [0, horizon_steps-1]. A
// knot_ratio of exactly 1 uses the uniform knots, so it matches them bitwise.
VectorXd makeKnots(const std::string& prefix,
                   int horizon_steps,
                   int degree,
                   int num_control_points,
                   double knot_ratio)
{
  if (!(knot_ratio > 0.0) || !std::isfinite(knot_ratio))
    throw std::invalid_argument(prefix
                                + "knot_ratio must be positive and finite.\n");
  if (knot_ratio == 1.0)
    return Parameterization::makeUniformClampedKnots(horizon_steps, degree,
                                                     num_control_points);
  validateHorizonSteps(horizon_steps);
  validateDegree(degree, horizon_steps);
  validateNumControls(num_control_points, horizon_steps, degree);

  const int num_intervals{num_control_points - degree};
  VectorXd knots{num_control_points + degree + 1};
  knots.head(degree + 1).setZero();
  double length{1.0};
  for (int i{degree}; i < degree + num_intervals; ++i, length *= knot_ratio)
    knots(i + 1) = knots(i) + length;
  knots.segment(degree + 1, num_intervals) *=
      (horizon_steps - 1) / knots(degree + num_intervals);
  knots.tail(degree + 1).setConstant(horizon_steps - 1); // exact, not round-off

  // short first (or last) intervals are fine as long as the sampled basis
  // keeps full column rank
  const std::string reason{rankDeficiency(
      degree, knots, computeBandStart(horizon_steps, degree, knots),
      computeBandWeights(horizon_steps, degree, knots))};
  if (!reason.empty()) {
    std::stringstream ss;
    ss << prefix << "knot_ratio = " << knot_ratio
       << " gives knot intervals too short for the samples: " << reason
       << " Use a knot_ratio closer to 1 or fewer control points.\n";
    throw std::invalid_argument(ss.str());
  }
  return knots;
}

} // namespace

Parameterization Parameterization::moveBlocking(int horizon_steps,
                                                int num_control_points,
                                                double knot_ratio)
{
  const int deg{0};
  const auto knots{makeKnots("[Parameterization::moveBlocking] ", horizon_steps,
                             deg, num_control_points, knot_ratio)};
  return Parameterization{horizon_steps, deg, knots};
}

Parameterization
Parameterization::moveBlocking(int horizon_steps,
                               const Ref<const VectorXd>& change_points)
{
  if (change_points.size() == 0) {
    throw std::invalid_argument(
        "[Parameterization::moveBlocking] change_points can not be empty.");
  }
  if (change_points.size() > horizon_steps) {
    throw std::invalid_argument(
        "[Parameterization::moveBlocking] change_points size must be less than "
        "or equal to horizon_steps.");
  }
  // horizon_steps - 1 is appended below to close the last block. A change point
  // there is still valid and gives the last sample its own block, since the
  // degree 0 span search never reads the final knot.
  if (change_points(ph::last) > horizon_steps - 1) {
    throw std::invalid_argument(
        "[Parameterization::moveBlocking] change_points must be less than "
        "or equal to horizon_steps - 1.");
  }
  const int deg{0};
  const int num_control_points = change_points.size();
  VectorXd knots{num_control_points + 1};
  knots << change_points, horizon_steps - 1;
  return Parameterization{horizon_steps, deg, knots};
}

Parameterization Parameterization::linearInterp(int horizon_steps,
                                                int num_control_points,
                                                double knot_ratio)
{
  const int deg{1};
  const auto knots{makeKnots("[Parameterization::linearInterp] ", horizon_steps,
                             deg, num_control_points, knot_ratio)};
  return Parameterization{horizon_steps, deg, knots};
}

Parameterization
Parameterization::linearInterp(int horizon_steps,
                               const Ref<const VectorXd>& endpoints)
{
  // static const std::string prefix{""};
  const int num_control_points = endpoints.size();
  if (num_control_points > horizon_steps)
    throw std::invalid_argument(
        "[Parameterization::linearInterp] "
        "Size of endpoints can not exceed horizon_steps.\n");
  if (num_control_points < 2)
    throw std::invalid_argument("[Parameterization::linearInterp] "
                                "Size of endpoints must be at least 2.\n");

  const int deg{1};
  VectorXd knots{num_control_points + 2};
  knots << 0.0, endpoints, horizon_steps - 1.0;
  return Parameterization{horizon_steps, deg, knots};
}

Parameterization Parameterization::bspline(int horizon_steps,
                                           int degree,
                                           int num_control_points,
                                           double knot_ratio)
{
  const auto knots{makeKnots("[Parameterization::bspline] ", horizon_steps,
                             degree, num_control_points, knot_ratio)};
  return Parameterization{horizon_steps, degree, knots};
}

Parameterization Parameterization::bspline(
    int horizon_steps, int degree, const Ref<const VectorXd>& active_knots)
{
  if (active_knots.size() < 2) {
    throw std::invalid_argument("[Parameterization::bspline] "
                                "Size of active_knots must be at least 2.\n");
  }

  const int num_control_points = active_knots.size() + degree - 1;
  VectorXd knots{num_control_points + degree + 1};
  knots.head(degree).setZero();
  knots.tail(degree).setConstant(horizon_steps - 1);
  knots(seq(degree, ph::last - degree)) = active_knots;
  return Parameterization{horizon_steps, degree, knots};
}

Parameterization Parameterization::fromBasis(int horizon_steps,
                                             const Ref<const MatrixXd>& basis)
{
  static const std::string prefix{"[Parameterization::fromBasis] "};
  validateHorizonSteps(horizon_steps);
  if (basis.rows() != horizon_steps)
    throw std::invalid_argument(prefix + "basis must have horizon_steps = "
                                + std::to_string(horizon_steps)
                                + " rows (one per sample), got "
                                + std::to_string(basis.rows()) + ".\n");
  if (basis.cols() < 1)
    throw std::invalid_argument(prefix
                                + "basis must have at least one column.\n");
  if (basis.cols() > horizon_steps)
    throw std::invalid_argument(
        prefix
        + "basis can not have more columns than rows (it would not "
          "have full column rank).\n");
  if (!basis.allFinite())
    throw std::invalid_argument(prefix + "basis entries must be finite.\n");

  // Rank by singular values, relative to the largest. The tolerance is looser
  // than round-off so nearly dependent columns are reported, but full rank
  // does not make the QP well conditioned (cond(P) grows like the square of
  // cond(basis)).
  constexpr double rank_tol{1e-9};
  const VectorXd sv{JacobiSVD<MatrixXd>{basis}.singularValues()};
  const double ratio{sv(0) > 0.0 ? sv(sv.size() - 1) / sv(0) : 0.0};
  if (!(ratio > rank_tol)) {
    std::stringstream ss;
    ss << std::setprecision(3) << prefix
       << "basis must have full column rank: its smallest singular value is "
       << ratio << " times its largest (at most " << rank_tol
       << "), so some combination of coefficients has (almost) no effect on "
       << "any sampled input. If the columns are only badly scaled, "
       << "orthonormalize them (e.g. by a QR decomposition). If they are "
       << "nearly dependent, orthonormalizing would turn the dependent "
       << "direction into a column of round-off; instead keep the directions "
       << "that affect the inputs: with the SVD basis = U S V^T, pass the "
       << "columns of U whose singular values exceed a tolerance (e.g. 1e-6 "
       << "times the largest).\n";
    throw std::invalid_argument(ss.str());
  }
  return Parameterization{horizon_steps, MatrixXd{basis}};
}

Parameterization::Parameterization(int horizon_steps, const MatrixXd& basis) :
    horizon_steps{horizon_steps},
    degree{-1},
    num_control_points{static_cast<int>(basis.cols())},
    knots{},
    band_width{static_cast<int>(basis.cols())},
    band_start{VectorXi::Zero(horizon_steps)},
    band_weights{basis.transpose()},
    type{Type::Custom}
{}

Parameterization::Parameterization(int horizon_steps,
                                   int degree,
                                   int num_control_points) :
    horizon_steps{horizon_steps},
    degree{degree},
    num_control_points{num_control_points},
    knots{makeUniformClampedKnots(horizon_steps, degree, num_control_points)},
    band_width{degree + 1},
    band_start{computeBandStart(horizon_steps, degree, knots)},
    band_weights{computeBandWeights(horizon_steps, degree, knots)},
    type{Type::BSpline}
{
  validateBasisRank(*this);
}

Parameterization::Parameterization(int horizon_steps,
                                   int degree,
                                   const Ref<const VectorXd>& knots) :
    horizon_steps{horizon_steps},
    degree{degree},
    num_control_points{int(knots.size()) - degree - 1},
    knots{validatedKnots(knots, horizon_steps, degree)},
    band_width{degree + 1},
    band_start{computeBandStart(horizon_steps, degree, this->knots)},
    band_weights{computeBandWeights(horizon_steps, degree, this->knots)},
    type{Type::BSpline}
{
  validateBasisRank(*this);
}

VectorXd Parameterization::evaluate(
    int input_dim,
    const Eigen::Ref<const Eigen::VectorXd>& control_points) const
{
  if (input_dim < 1)
    throw std::invalid_argument(
        "[Parameterization::evaluate] input_dim must be positive.");
  if (control_points.size() != num_control_points * input_dim)
    throw std::invalid_argument(
        "[Parameterization::evaluate] "
        "Size of control_points must be input_dim*num_control_points");

  VectorXd u_traj{input_dim * horizon_steps};
  Map<MatrixXd> map{u_traj.data(), input_dim, horizon_steps};
  Map<const MatrixXd> ctrls{control_points.data(), input_dim,
                            num_control_points};
  for (int k{0}; k < horizon_steps; ++k) {
    map.col(k).noalias() =
        ctrls.middleCols(band_start(k), band_width) * band_weights.col(k);
  }
  return u_traj;
}

Eigen::VectorXd Parameterization::makeClampedKnots(int horizon_steps,
                                                   int degree,
                                                   int num_control_points,
                                                   double knot_ratio)
{
  return makeKnots("[Parameterization::makeClampedKnots] ", horizon_steps,
                   degree, num_control_points, knot_ratio);
}

Eigen::VectorXd Parameterization::makeUniformClampedKnots(
    int horizon_steps, int degree, int num_control_points)
{
  validateHorizonSteps(horizon_steps);
  validateDegree(degree, horizon_steps);
  validateNumControls(num_control_points, horizon_steps, degree);

  Eigen::VectorXd knots{num_control_points + degree + 1};
  // Clamped B-spline repeats at head and tail by degree
  knots.head(degree).setZero();
  knots.tail(degree).setConstant(horizon_steps - 1);

  // Active knots are uniformly spaced in between
  const int num_active_knots = knots.size() - 2 * degree;
  knots(seq(degree, ph::last - degree)) =
      ArrayXd::LinSpaced(num_active_knots, 0, horizon_steps - 1);
  return knots;
}

std::vector<int> Parameterization::inputSaturationSamples() const
{
  // degree 0: each sampled input equals one control point, so bounding the
  // control points is already exact
  if (type == Type::BSpline && degree == 0)
    return {};

  // A custom basis: every sample whose basis row is not negligible. A decaying
  // basis (e.g. Laguerre functions) has rows far below round-off late in a
  // long horizon (about 1e-60), which add nothing to the feasible set unless
  // the coefficients exceed the input limits by 1/prune_tol, but would give
  // the constraint matrix a dynamic range that hurts the solver's scaling.
  std::vector<int> samples;
  if (type == Type::Custom) {
    constexpr double prune_tol{1e-9};
    const ArrayXd row_norms{band_weights.colwise().norm().transpose()};
    const double keep_above{prune_tol * row_norms.maxCoeff()};
    for (int k{0}; k < horizon_steps; ++k)
      if (row_norms(k) > keep_above)
        samples.push_back(k);
    return samples;
  }

  // degree > 1: every sample
  if (degree > 1) {
    samples.resize(horizon_steps);
    std::iota(samples.begin(), samples.end(), 0);
    return samples;
  }

  // Degree 1: the spline is linear on each knot interval [t_i, t_{i+1}), so
  // bounding the first and last sample of every interval, ceil(t) - 1 and
  // ceil(t) for every active knot t, is exactly equivalent to bounding every
  // sample (at most 2*num_control_points-2 samples). Where the spline is
  // continuous (a simple knot), the sample at an integer knot also bounds the
  // sample before it, so only that sample is used. Round-off from integers
  // is treated as an integer there. A repeated knot makes the spline jump, so
  // both samples are needed. Active knots are knots(1), ..., knots(size-2) and
  // span [0, T-1].
  constexpr double snap_tol{1e-9};
  const Index first_active{1}, last_active{knots.size() - 2};
  samples.reserve(2 * (knots.size() - 2));
  for (Index i{first_active}; i <= last_active; ++i) {
    const double t{knots(i)};
    const bool repeated{(i > first_active && knots(i - 1) == t)
                        || (i < last_active && knots(i + 1) == t)};
    const double t_round{std::round(t)};
    if (!repeated && std::abs(t - t_round) <= snap_tol) {
      samples.push_back(static_cast<int>(t_round));
    } else {
      const int after{static_cast<int>(std::ceil(t))};
      if (after - 1 >= 0)
        samples.push_back(after - 1);
      if (after <= horizon_steps - 1)
        samples.push_back(after);
    }
  }
  std::sort(samples.begin(), samples.end());
  samples.erase(std::unique(samples.begin(), samples.end()), samples.end());
  return samples;
}

void Parameterization::refineKnots(const int max_interval,
                                   VectorXd& refined_knots,
                                   VectorXi& R_start,
                                   MatrixXd& R_weights,
                                   std::vector<int>& sampled) const
{
  assert(type == Type::BSpline && max_interval > 0);
  const int p{degree}, n{num_control_points};
  const VectorXd& t{knots};

  // Split every nonempty active knot interval [t_i, t_{i+1}] (i = p, ...,
  // n-1) into max(2, ceil(length / max_interval)) equal parts, so every
  // interval is split at least once and none is longer than max_interval.
  // Lengths within round-off of a multiple of max_interval are not split
  // further.
  const double max_length{max_interval * (1.0 + 1e-9)};
  std::vector<double> tau(t.data(), t.data() + p + 1);
  for (int i{p}; i < n; ++i) {
    const double length{t(i + 1) - t(i)};
    if (length > 0.0) {
      const int parts{
          std::max(2, static_cast<int>(std::ceil(length / max_length)))};
      for (int q{1}; q < parts; ++q)
        tau.push_back(t(i) + length * q / parts);
    }
    tau.push_back(t(i + 1));
  }
  tau.insert(tau.end(), t.data() + n + 1, t.data() + t.size());
  refined_knots =
      Map<const VectorXd>{tau.data(), static_cast<Index>(tau.size())};
  const int n_ref{static_cast<int>(tau.size()) - p - 1};

  // Refined control point j is the blossom of the spline at
  // (tau_{j+1}, ..., tau_{j+p}), evaluated on the polynomial piece of any
  // nonempty refined interval [tau_l, tau_{l+1}) in its support inside the
  // active range (j <= l <= j+p, p <= l <= n_ref-1). That piece lies in the
  // original interval [t_mu, t_{mu+1}), so the blossom is de Boor's algorithm
  // on control points mu-p, ..., mu with the argument tau_{j+r} at level r
  // (the Oslo algorithm). Each refined control point is a combination of at
  // most p+1 consecutive control points, so R is banded.
  R_start.resize(n_ref);
  R_weights.resize(p + 1, n_ref);
  MatrixXd D{p + 1, p + 1}; // column c: d_{mu-p+c} on control points mu-p..mu
  for (int j{0}; j < n_ref; ++j) {
    int l{std::max(j, p)};
    const int l_max{std::min(j + p, n_ref - 1)};
    while (l < l_max && !(tau[l] < tau[l + 1]))
      ++l;
    assert(tau[l] < tau[l + 1]);
    const int mu{
        static_cast<int>(std::upper_bound(t.data(), t.data() + n, tau[l])
                         - t.data())
        - 1};
    assert(mu >= p && mu < n);
    D.setIdentity();
    for (int r{1}; r <= p; ++r) {
      const double x{tau[j + r]};
      for (int c{p}; c >= r; --c) {
        const int i{mu - p + c};
        const double alpha{(x - t(i)) / (t(i + p + 1 - r) - t(i))};
        D.col(c) = (1.0 - alpha) * D.col(c - 1) + alpha * D.col(c);
      }
    }
    R_start(j) = mu - p;
    R_weights.col(j) = D.col(p);
  }

  // The refined control points that some sample depends on (each sampled
  // input is a convex combination of them)
  std::vector<bool> used(n_ref, false);
  const VectorXd tau_vec{refined_knots};
  const VectorXi start{computeBandStart(horizon_steps, p, tau_vec)};
  const MatrixXd weights{computeBandWeights(horizon_steps, p, tau_vec)};
  for (int k{0}; k < horizon_steps; ++k)
    for (int i{0}; i <= p; ++i)
      if (weights(i, k) != 0.0)
        used[start(k) + i] = true;
  sampled.clear();
  for (int j{0}; j < n_ref; ++j)
    if (used[j])
      sampled.push_back(j);
}

std::ostream& operator<<(std::ostream& os, const Parameterization& param)
{
  if (param.type == Parameterization::Type::Custom) {
    return os << "Parameterization(type=custom, horizon_steps="
              << param.horizon_steps
              << ", num_control_points=" << param.num_control_points << ")";
  }

  os << "Parameterization(horizon_steps=" << param.horizon_steps
     << ", degree=" << param.degree
     << ", num_control_points=" << param.num_control_points << ", knots=[";

  const IOFormat fmt{StreamPrecision, DontAlignCols, ", ", ", ", "", ""};
  const Index knots_size{param.knots.size()};
  if (knots_size > 12) {
    os << param.knots.head(5).format(fmt);
    os << ", ..., ";
    os << param.knots.tail(5).format(fmt);
  } else {
    os << param.knots.format(fmt);
  }
  os << "])";
  return os;
}

} // namespace affine_mpc
