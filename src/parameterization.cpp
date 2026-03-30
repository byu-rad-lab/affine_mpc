#include "affine_mpc/parameterization.hpp"

#include <Eigen/Core>
#include <sstream>
#include <stdexcept>
#include <unsupported/Eigen/Splines>

namespace affine_mpc {

using namespace Eigen;
namespace ph = Eigen::placeholders;

namespace { // utilities for this file

void validateHorizonSteps(const int horizon_steps)
{
  if (horizon_steps < 1)
    throw std::invalid_argument("[Parameterization::validateHorizonSteps] "
                                "horizon_steps must be positive.\n");
}

void validateDegree(const int degree, const int horizon_steps)
{
  static const std::string prefix{"[Parameterization::validateDegree] "};
  if (degree < 0)
    throw std::invalid_argument(prefix + "degree can not be negative.\n");
  if (degree >= horizon_steps)
    throw std::invalid_argument(prefix
                                + "degree must be less than horizon_steps.\n");
}

void validateNumControls(const int num_control_points,
                         const int horizon_steps,
                         const int degree)
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
                   const int horizon_steps,
                   const int degree)
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
    ss << "[Parameterization::validateKnots] First active knot must equal zero "
       << "(representing input at current time step, k=0).\n    "
       << "Got " << knots(degree) << std::endl;
    throw std::invalid_argument(ss.str());
  }
  if (knots(ph::last - degree) != horizon_steps - 1) {
    std::stringstream ss;
    ss << "[Parameterization::validateKnots] "
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

} // namespace

Parameterization Parameterization::moveBlocking(const int horizon_steps,
                                                const int num_control_points)
{
  const int deg{0};
  const auto knots{
      makeUniformClampedKnots(horizon_steps, deg, num_control_points)};
  return Parameterization{horizon_steps, deg, knots};
}

Parameterization
Parameterization::moveBlocking(const int horizon_steps,
                               const Ref<const VectorXd>& change_points)
{
  if (change_points.size() > horizon_steps) {
    throw std::invalid_argument(
        "[Parameterization::moveBlocking] change_points size must be less than "
        "or equal to horizon_steps.");
  }
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

Parameterization Parameterization::linearInterp(const int horizon_steps,
                                                const int num_control_points)
{
  const int deg{1};
  const auto knots{
      makeUniformClampedKnots(horizon_steps, deg, num_control_points)};
  return Parameterization{horizon_steps, deg, knots};
}

Parameterization
Parameterization::linearInterp(const int horizon_steps,
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

Parameterization Parameterization::bspline(const int horizon_steps,
                                           const int degree,
                                           const int num_control_points)
{
  const auto knots{
      makeUniformClampedKnots(horizon_steps, degree, num_control_points)};
  return Parameterization{horizon_steps, degree, knots};
}

Parameterization
Parameterization::bspline(const int horizon_steps,
                          const int degree,
                          const Ref<const VectorXd>& active_knots)
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

Parameterization::Parameterization(const int horizon_steps,
                                   const int degree,
                                   const int num_control_points) :
    horizon_steps{horizon_steps},
    degree{degree},
    num_control_points{num_control_points},
    knots{makeUniformClampedKnots(horizon_steps, degree, num_control_points)}
{}

Parameterization::Parameterization(const int horizon_steps,
                                   const int degree,
                                   const Ref<const VectorXd>& knots) :
    horizon_steps{horizon_steps},
    degree{degree},
    num_control_points{int(knots.size()) - degree - 1},
    knots{knots}
{
  validateHorizonSteps(horizon_steps);
  validateDegree(degree, horizon_steps);
  validateKnots(knots, horizon_steps, degree);
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
        "Size of control_points must be input_dim*horizon_steps");

  using Spline1d = Spline<double, 1>;
  VectorXd u_traj{input_dim * horizon_steps};
  u_traj.setZero();

  Map<MatrixXd> map{u_traj.data(), input_dim, horizon_steps};
  Map<const MatrixXd> ctrls{control_points.data(), input_dim,
                            num_control_points};
  const int order{degree + 1};
  VectorXd weights{order};

  for (int k{0}; k < horizon_steps; ++k) {
    const double t = k;
    const int idx = Spline1d::Span(t, degree, knots) - degree;
    weights = Spline1d::BasisFunctions(t, degree, knots);
    for (int i{0}; i < order; ++i)
      map.col(k).noalias() = ctrls.middleCols(idx, order) * weights;
  }
  return u_traj;
}

Eigen::VectorXd Parameterization::makeUniformClampedKnots(
    const int horizon_steps, const int degree, const int num_control_points)
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

} // namespace affine_mpc
