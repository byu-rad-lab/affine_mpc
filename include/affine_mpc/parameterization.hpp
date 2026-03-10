#ifndef AFFINE_MPC_PARAMETERIZATION_HPP
#define AFFINE_MPC_PARAMETERIZATION_HPP

#include <Eigen/Core>

/**
 * @file parameterization.hpp
 * @brief Defines the Parameterization class for parameterizing the MPC input
 *   trajectory with a B-spline to reduce the number of design variables in the
 *   optimization and smooth the input trajectory.
 */

namespace affine_mpc {

/**
 * @class Parameterization
 * @brief Encapsulates B-spline input trajectory parameterization for MPC.
 *
 * Provides constructors and factory methods for generating knot vectors and
 * metadata to map control points to per-step input trajectories over the
 * prediction horizon.
 *
 * The preferred usage is via the static factory methods for common
 * parameterizations.
 */
class Parameterization
{
public:
  /**
   * @brief Direct constructor for uniform clamped B-spline parameterization.
   * @param horizon_steps Number of discrete time steps in the horizon.
   * @param num_control_points Number of B-spline control points.
   * @param degree B-spline polynomial degree.
   *
   * For common cases, prefer the named factory methods.
   */
  Parameterization(const int horizon_steps,
                   const int num_control_points,
                   const int degree);

  /**
   * @brief Direct constructor for advanced use cases with custom knot vector
   *   (e.g. unclamped B-splines).
   * @param horizon_steps Number of discrete time steps in the horizon.
   * @param num_control_points Number of B-spline control points.
   * @param degree B-spline polynomial degree.
   * @param knots Full knot vector (size = num_control_points + degree + 1).
   */
  Parameterization(const int horizon_steps,
                   const int num_control_points,
                   const int degree,
                   const Eigen::Ref<const Eigen::VectorXd>& knots);

  ~Parameterization() = default;

  /**
   * @brief Validates the knot vector for correctness.
   * @param error_msg Populated with error description if validation fails.
   * @return True if knots are valid, false otherwise.
   */
  bool validateKnots(std::string& error_msg) const;

  /**
   * @brief Factory method for uniform move-blocking parameterization.
   * @param horizon_steps Number of discrete time steps in the horizon.
   * @param num_control_points Number of control points.
   * @return Parameterization instance.
   *
   * Piecewise constant inputs set from previous change point.
   */
  static Parameterization moveBlocking(const int horizon_steps,
                                       const int num_control_points);

  /**
   * @brief Factory method for move-blocking parameterization with custom change
   *   points.
   * @param horizon_steps Number of discrete time steps in the horizon.
   * @param change_points Vector of change point locations.
   * @return Parameterization instance.
   */
  static Parameterization
  moveBlocking(const int horizon_steps,
               const Eigen::Ref<const Eigen::VectorXd>& change_points);

  /**
   * @brief Factory method for uniform linear interpolation parameterization.
   * @param horizon_steps Number of discrete time steps in the horizon.
   * @param num_control_points Number of control points.
   * @return Parameterization instance.
   *
   * Change points are evenly spaced between 0 and horizon_steps - 1.
   */
  static Parameterization linearInterp(const int horizon_steps,
                                       const int num_control_points);

  /**
   * @brief Factory method for linear interpolation parameterization with custom
   *   change points.
   * @param horizon_steps Number of discrete time steps in the horizon.
   * @param endpoints Vector of linear segment endpoint locations. Must include
   *   0 and horizon_steps - 1.
   * @return Parameterization instance.
   */
  static Parameterization
  linearInterp(const int horizon_steps,
               const Eigen::Ref<const Eigen::VectorXd>& endpoints);

  /**
   * @brief Factory method for uniform clamped B-spline parameterization.
   * @param horizon_steps Number of discrete time steps in the horizon.
   * @param num_control_points Number of control points.
   * @param degree B-spline polynomial degree.
   * @return Parameterization instance.
   */
  static Parameterization bspline(const int horizon_steps,
                                  const int num_control_points,
                                  const int degree);

  /**
   * @brief Factory method for clamped B-spline parameterization with custom
   *   active knots.
   * @param horizon_steps Number of discrete time steps in the horizon.
   * @param num_control_points Number of control points.
   * @param degree B-spline polynomial degree.
   * @param active_knots Vector of active knot locations.
   * @return Parameterization instance.
   */
  static Parameterization
  bspline(const int horizon_steps,
          const int num_control_points,
          const int degree,
          const Eigen::Ref<const Eigen::VectorXd>& active_knots);

  /// Full knot vector of size num_control_points + degree + 1.
  Eigen::VectorXd knots;

  /// Number of discrete time steps in the horizon.
  const int horizon_steps;

  /// Number of B-spline control points.
  const int num_control_points;

  /// B-spline polynomial degree.
  const int degree;
};

} // namespace affine_mpc

#endif // AFFINE_MPC_PARAMETERIZATION_HPP
