#include "affine_mpc/parameterization.hpp"

#include <Eigen/Core>
#include <algorithm>
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

  const int num_active_knots = knots.size() - 2 * degree;
  // deg 0 can repeat the last knot (last one isn't really active)
  const int size{degree == 0 ? num_active_knots - 2 : num_active_knots - 1};
  if (size > 0 && diff(seqN(degree, size)).minCoeff() == 0.0)
    throw std::invalid_argument(
        "[Parameterization::validateKnots] active knots can not be repeated "
        "(technically a spline can repeat knots, but it causes discontinuities "
        "that essentially leave some control points unused and wastes "
        "optimization effort in this context).\n");
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

// A basis function that is zero at every sample leaves its control point
// without any effect on the inputs, so the QP is singular in that direction.
// This happens when knots closer than one sample leave no sample inside a
// support (degree 0: a block [t_i, t_i+1) that contains no integer).
void validateInfluence(const Parameterization& p)
{
  VectorXd influence{VectorXd::Zero(p.num_control_points)};
  for (int k{0}; k < p.horizon_steps; ++k)
    influence.segment(p.band_start(k), p.band_width) += p.band_weights.col(k);
  for (int i{0}; i < p.num_control_points; ++i) {
    if (influence(i) == 0.0) {
      std::stringstream ss;
      ss << std::setprecision(17) << "[Parameterization::validateInfluence] "
         << "control point " << i
         << " has no influence on any sampled input: no sample time k = 0, "
         << "..., horizon_steps-1 lies inside the support of its basis "
         << "function [" << p.knots(i) << ", " << p.knots(i + p.degree + 1)
         << "]. Spread the knots so every support contains a sample.\n";
      throw std::invalid_argument(ss.str());
    }
  }
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

  // the shortest interval is the first for knot_ratio > 1 and the last for < 1
  const int shortest{knot_ratio > 1.0 ? degree : degree + num_intervals - 1};
  const double min_length{knots(shortest + 1) - knots(shortest)};
  if (min_length < 1.0) {
    std::stringstream ss;
    ss << std::setprecision(17) << prefix
       << "shortest knot interval must be at least one sample. Got "
       << min_length << " with knot_ratio = " << knot_ratio
       << "; use a knot_ratio closer to 1 or fewer control points.\n";
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

Parameterization::Parameterization(int horizon_steps,
                                   int degree,
                                   int num_control_points) :
    horizon_steps{horizon_steps},
    degree{degree},
    num_control_points{num_control_points},
    knots{makeUniformClampedKnots(horizon_steps, degree, num_control_points)},
    band_width{degree + 1},
    band_start{computeBandStart(horizon_steps, degree, knots)},
    band_weights{computeBandWeights(horizon_steps, degree, knots)}
{
  validateInfluence(*this);
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
    band_weights{computeBandWeights(horizon_steps, degree, this->knots)}
{
  validateInfluence(*this);
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
  if (degree == 0)
    return {};

  // degree > 1: every sample
  std::vector<int> samples;
  if (degree > 1) {
    samples.resize(horizon_steps);
    std::iota(samples.begin(), samples.end(), 0);
    return samples;
  }

  // Degree 1: floor(t) and ceil(t) of every active knot t, without duplicates.
  // The spline is linear between knots, so every other sample lies between the
  // first and last samples of its segment, and bounding these (at most
  // 2*num_control_points-2) samples is exactly equivalent to bounding all of
  // them. Active knots can not repeat, so the spline is continuous. Active
  // knots are knots(1), ..., knots(size-2) and span [0, T-1].
  constexpr double snap_tol{1e-9}; // treat round-off from integers as integers
  samples.reserve(2 * (knots.size() - 2));
  for (Index i{1}; i < knots.size() - 1; ++i) {
    const double t{knots(i)};
    const double t_round{std::round(t)};
    if (std::abs(t - t_round) <= snap_tol) {
      samples.push_back(static_cast<int>(t_round));
    } else {
      samples.push_back(static_cast<int>(std::floor(t)));
      samples.push_back(static_cast<int>(std::ceil(t)));
    }
  }
  std::sort(samples.begin(), samples.end());
  samples.erase(std::unique(samples.begin(), samples.end()), samples.end());
  return samples;
}

std::ostream& operator<<(std::ostream& os, const Parameterization& param)
{
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
