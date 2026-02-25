#include "affine_mpc/parameterization.hpp"

#include <Eigen/Core>
#include <sstream>
#include <stdexcept>

namespace affine_mpc {

using namespace Eigen;
namespace ph = Eigen::placeholders;

Parameterization::Parameterization(const int horizon_steps,
                                   const int num_control_points,
                                   const int degree) :
    knots{num_control_points + degree + 1},
    horizon_steps{horizon_steps},
    num_control_points{num_control_points},
    degree{degree}
{
  knots.head(degree).setZero();
  knots.tail(degree).setConstant(horizon_steps - 1);

  const int num_internal_knots = knots.size() - 2 * degree;
  knots(seq(degree, ph::last - degree)) =
      ArrayXd::LinSpaced(num_internal_knots, 0, horizon_steps - 1);
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
  const int size = this->knots.size();
  if (knots.size() != size) {
    std::stringstream ss;
    ss << "[Parameterization Constructor] Invalid knots size:\n    "
       << "Expected " << size << "(num_control_points + degree + 1)\n    "
       << "Got " << knots.size() << std::endl;
    throw std::invalid_argument(ss.str());
  }
  if ((knots(seq(1, ph::last)) - knots(seq(0, ph::last - 1))).minCoeff() < 0)
    throw std::invalid_argument(
        "[Parameterization Constructor] knots must be non-decreasing.\n");
  if (knots.head(degree + 1).minCoeff() > 0.0)
    throw std::invalid_argument(
        "[Parameterization Constructor] First degree+1 knots must be less than "
        "or equal to zero so that spline can be evaluated at 0.\n");
  if (knots.tail(degree + 1).maxCoeff() < horizon_steps - 1)
    throw std::invalid_argument(
        "[Parameterization Constructor] Last degree+1 knots must be greater "
        "than or equal to horizon_steps - 1 so that spline can be evaluated at "
        "horizon_steps - 1.\n");
  if (knots(degree) != 0.0)
    throw std::invalid_argument(
        "[Parameterization Constructor] First internal knot must be equal to "
        "zero representing input at current time step (k=0).\n");
  if (knots(ph::lastp1 - degree) != horizon_steps - 1)
    throw std::invalid_argument(
        "[Parameterization Constructor] Last internal knot must be equal to "
        "horizon_steps - 1 representing final input.\n");

  this->knots = knots;
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
  const int num_control_points = change_points.size() + 1;
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
                          const Ref<const VectorXd>& internal_knots)
{
  if (internal_knots.size() != num_control_points - degree - 1) {
    std::stringstream ss;
    ss << "[Parameterization::bspline] Invalid internal_knots size:\n"
       << "    Expected " << num_control_points - degree - 1 << "\n"
       << "    Got " << internal_knots.size() << std::endl;
    throw std::invalid_argument(ss.str());
  }

  VectorXd knots{num_control_points + degree + 1};
  knots.head(degree).setZero();
  knots.tail(degree).setConstant(horizon_steps - 1);
  knots(seq(degree, ph::last - degree)) = internal_knots;
  Parameterization param{horizon_steps, num_control_points, degree, knots};
  return param;
}

} // namespace affine_mpc
