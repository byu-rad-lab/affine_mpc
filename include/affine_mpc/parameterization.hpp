#ifndef PARAMETERIZATION_HPP
#define PARAMETERIZATION_HPP

#include <Eigen/Core>

namespace affine_mpc {

class Parameterization
{
public:
  /// Direct constructor for advanced use cases (e.g. unclamped B-splines).
  /// For common cases, prefer the named factory methods below.
  Parameterization(const int horizon_steps,
                   const int num_control_points,
                   const int degree);
  Parameterization(const int horizon_steps,
                   const int num_control_points,
                   const int degree,
                   const Eigen::Ref<const Eigen::VectorXd>& knots);
  ~Parameterization() = default;

  /// Factory method for uniform move-blocking parameterization
  /// (i.e. piecewise constant inputs set from previous change point)
  static Parameterization moveBlocking(const int horizon_steps,
                                       const int num_control_points);
  /// Factory method for move-blocking parameterization
  /// (i.e. piecewise constant inputs set from previous change point)
  static Parameterization
  moveBlocking(const int horizon_steps,
               const Eigen::Ref<const Eigen::VectorXd>& change_points);

  /// Factory method for uniform linear interpolation parameterization
  /// (i.e. change points are evenly spaced between 0 and horizon_steps - 1)
  static Parameterization linearInterp(const int horizon_steps,
                                       const int num_control_points);
  /// Factory method for linear interpolation parameterization
  static Parameterization
  linearInterp(const int horizon_steps,
               const Eigen::Ref<const Eigen::VectorXd>& change_points);

  /// Factory method for uniform clamped B-spline parameterization
  static Parameterization bspline(const int horizon_steps,
                                  const int num_control_points,
                                  const int degree);
  /// Factory method for clamped B-spline parameterization
  static Parameterization
  bspline(const int horizon_steps,
          const int num_control_points,
          const int degree,
          const Eigen::Ref<const Eigen::VectorXd>& active_knots);

  // public member variables
  Eigen::VectorXd knots;
  int horizon_steps;
  int num_control_points;
  int degree;
};

} // namespace affine_mpc

#endif // PARAMETERIZATION_HPP
