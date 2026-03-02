#include "affine_mpc/parameterization.hpp"

#include <Eigen/Core>
#include <sstream>
#include <stdexcept>

namespace affine_mpc {

using namespace Eigen;
namespace ph = Eigen::placeholders;

void validateConstructorParams(const int horizon_steps,
                               const int num_control_points,
                               const int degree)
{
  if (horizon_steps < 1) {
    std::stringstream ss;
    ss << "[Parameterization] horizon_steps must be at least 1.\n"
       << "    Got " << horizon_steps << std::endl;
    throw std::invalid_argument(ss.str());
  }
  if (num_control_points < 1 || num_control_points > horizon_steps) {
    std::stringstream ss;
    ss << "[Parameterization] "
       << "num_control_points must be in the range (1, horizon_steps].\n    "
       << "Got " << num_control_points
       << " with horizon_steps = " << horizon_steps << std::endl;
    throw std::invalid_argument(ss.str());
  }
  if (degree < 0 || degree > num_control_points - 1) {
    std::stringstream ss;
    ss << "[Parameterization] "
       << "degree must be in the range [0, num_control_points - 1].\n    Got "
       << degree << " with num_control_points = " << num_control_points
       << std::endl;
    throw std::invalid_argument(ss.str());
  }
}

void validateKnotValues(const Ref<const VectorXd>& knots,
                        const int horizon_steps,
                        const int degree)
{
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
       << "Last active knot must equal horizon_steps - 1 "
       << "(representing final input).\n    Got "
       << knots(knots.size() - degree - 1)
       << " with horizon_steps = " << horizon_steps << std::endl;
    throw std::invalid_argument(ss.str());
  }
  if ((knots(seq(1, ph::last)) - knots(seq(0, ph::last - 1))).minCoeff() < 0)
    throw std::invalid_argument(
        "[Parameterization::validateKnots] knots must be non-decreasing.\n");
  // may want to add more checks (e.g. max multiplicity of knots)
}

Parameterization::Parameterization(const int horizon_steps,
                                   const int num_control_points,
                                   const int degree) :
    knots{num_control_points + degree + 1},
    horizon_steps{horizon_steps},
    num_control_points{num_control_points},
    degree{degree}
{
  // Validate parameters first
  validateConstructorParams(horizon_steps, num_control_points, degree);

  // Clamped B-spline repeats at head and tail by degree
  knots.head(degree).setZero();
  knots.tail(degree).setConstant(horizon_steps - 1);

  // Active knots are uniformly spaced in between
  const int num_active_knots = knots.size() - 2 * degree;
  knots(seq(degree, ph::last - degree)) =
      ArrayXd::LinSpaced(num_active_knots, 0, horizon_steps - 1);
}

Parameterization::Parameterization(const int horizon_steps,
                                   const int num_control_points,
                                   const int degree,
                                   const Ref<const VectorXd>& knots) :
    knots{num_control_points + degree + 1},
    horizon_steps{horizon_steps},
    num_control_points{num_control_points},
    degree{degree}
{
  // Validate parameters first
  validateConstructorParams(horizon_steps, num_control_points, degree);

  const int size = this->knots.size();
  if (knots.size() != size) {
    std::stringstream ss;
    ss << "[Parameterization] Invalid knots size:\n    "
       << "Expected " << size << " (num_control_points + degree + 1)\n    "
       << "Got " << knots.size() << std::endl;
    throw std::invalid_argument(ss.str());
  }

  // Validate knot values
  validateKnotValues(knots, horizon_steps, degree);

  // All checks passed, assign member variable
  this->knots = knots;
}

bool Parameterization::validateKnots(std::string& error_msg) const
{
  try {
    validateKnotValues(knots, horizon_steps, degree);
    return true;
  } catch (const std::invalid_argument& e) {
    error_msg = e.what();
    return false;
  }
}

Parameterization Parameterization::moveBlocking(const int horizon_steps,
                                                const int num_control_points)
{
  Parameterization param{horizon_steps, num_control_points, 0};
  return param;
}

Parameterization
Parameterization::moveBlocking(const int horizon_steps,
                               const Ref<const VectorXd>& change_points)
{
  const int num_control_points = change_points.size() - 1;
  Parameterization param{horizon_steps, num_control_points, 0, change_points};
  return param;
}

Parameterization Parameterization::linearInterp(const int horizon_steps,
                                                const int num_control_points)
{
  Parameterization param{horizon_steps, num_control_points, 1};
  return param;
}

Parameterization
Parameterization::linearInterp(const int horizon_steps,
                               const Ref<const VectorXd>& change_points)
{
  const int num_control_points = change_points.size();
  if (change_points.size() > horizon_steps) {
    throw std::invalid_argument(
        "[Parameterization::linearInterp] change_points size must be less than "
        "or equal to horizon_steps.");
  }
  VectorXd knots{num_control_points + 2};
  knots << 0.0, change_points, horizon_steps - 1.0;
  Parameterization param{horizon_steps, num_control_points, 1, knots};
  return param;
}

Parameterization Parameterization::bspline(const int horizon_steps,
                                           const int num_control_points,
                                           const int degree)
{
  Parameterization param{horizon_steps, num_control_points, degree};
  return param;
}

Parameterization
Parameterization::bspline(const int horizon_steps,
                          const int num_control_points,
                          const int degree,
                          const Ref<const VectorXd>& active_knots)
{
  if (active_knots.size() != num_control_points - degree + 1) {
    std::stringstream ss;
    ss << "[Parameterization::bspline] Invalid active_knots size:\n"
       << "    Expected " << num_control_points - degree + 1 << "\n"
       << "    Got " << active_knots.size() << std::endl;
    throw std::invalid_argument(ss.str());
  }

  VectorXd knots{num_control_points + degree + 1};
  knots.head(degree).setZero();
  knots.tail(degree).setConstant(horizon_steps - 1);
  knots(seq(degree, ph::last - degree)) = active_knots;
  Parameterization param{horizon_steps, num_control_points, degree, knots};
  return param;
}

} // namespace affine_mpc
