#ifndef AFFINE_MPC_PARAMETERIZATION_HPP
#define AFFINE_MPC_PARAMETERIZATION_HPP

/**
 * @file parameterization.hpp
 * @brief Defines the Parameterization class for parameterizing the MPC input
 *   trajectory with a B-spline (or a custom basis) to reduce the number of
 *   design variables in the optimization and smooth the input trajectory.
 */

#include <Eigen/Core>
#include <iosfwd>
#include <vector>

namespace affine_mpc {

namespace detail {
class VectorParameterization;
}

/**
 * @class Parameterization
 * @brief Encapsulates B-spline input trajectory parameterization for MPC.
 *
 * Provides constructors and factory methods for generating knot vectors and
 * metadata to map control points to per-step input trajectories over the
 * prediction horizon. fromBasis() instead takes any sampled basis matrix
 * (e.g. Laguerre functions), whose coefficients play the role of the control
 * points.
 *
 * The preferred usage is via the static factory methods for common
 * parameterizations.
 */
class Parameterization
{
public:
  /// Kind of basis. Some MPC features rely on B-spline properties (see
  /// fromBasis()).
  enum class Type
  {
    BSpline, ///< B-spline from knots (move blocking, linear interpolation, ...)
    Custom   ///< Sampled basis matrix given to fromBasis()
  };

  /**
   * @brief Factory method for move-blocking parameterization with uniform or
   *   geometrically spaced blocks.
   * @param horizon_steps Number of discrete time steps in the horizon.
   * @param num_control_points Number of B-spline control points. Cannot be
   *   greater than horizon_steps.
   * @param knot_ratio Ratio of consecutive knot interval lengths (positive):
   *   the intervals are proportional to 1, knot_ratio, knot_ratio^2, ... and
   *   span [0, horizon_steps - 1]. The default of 1 gives uniform knots. A
   *   ratio above 1 puts short intervals (more flexibility) at the start of
   *   the horizon and a ratio below 1 puts them at the end.
   * @throws std::invalid_argument if the blocks are so short that some block
   *   has no sample of its own (the sampled basis would not have full column
   *   rank), or if any argument is invalid.
   * @return Parameterization instance.
   *
   * Piecewise constant inputs set from previous change point.
   */
  static Parameterization moveBlocking(int horizon_steps,
                                       int num_control_points,
                                       double knot_ratio = 1.0);

  /**
   * @brief Factory method for move-blocking parameterization with custom change
   *   points.
   * @param horizon_steps Number of discrete time steps in the horizon.
   * @param change_points Increasing start times of the blocks. The first must
   *   be 0 and the last at most horizon_steps - 1. horizon_steps - 1 is
   *   appended to close the last block, so 0, 1, ..., horizon_steps - 1 gives
   *   one control point per step.
   * @return Parameterization instance.
   */
  static Parameterization
  moveBlocking(int horizon_steps,
               const Eigen::Ref<const Eigen::VectorXd>& change_points);

  /**
   * @brief Factory method for linear interpolation parameterization with
   *   uniform or geometrically spaced endpoints.
   * @param horizon_steps Number of discrete time steps in the horizon.
   * @param num_control_points Number of B-spline control points. Cannot be
   *   greater than horizon_steps.
   * @param knot_ratio Ratio of consecutive knot interval lengths (positive):
   *   the intervals are proportional to 1, knot_ratio, knot_ratio^2, ... and
   *   span [0, horizon_steps - 1]. The default of 1 gives uniform knots. A
   *   ratio above 1 puts short intervals (more flexibility) at the start of
   *   the horizon and a ratio below 1 puts them at the end.
   * @throws std::invalid_argument if the segments are so short that the
   *   sampled basis does not have full column rank, or if any argument is
   *   invalid.
   * @return Parameterization instance.
   *
   * Endpoints span 0 to horizon_steps - 1.
   */
  static Parameterization linearInterp(int horizon_steps,
                                       int num_control_points,
                                       double knot_ratio = 1.0);

  /**
   * @brief Factory method for linear interpolation parameterization with custom
   *   change points.
   * @param horizon_steps Number of discrete time steps in the horizon.
   * @param endpoints Vector of linear segment endpoint locations. Must include
   *   0 and horizon_steps - 1.
   * @return Parameterization instance.
   */
  static Parameterization
  linearInterp(int horizon_steps,
               const Eigen::Ref<const Eigen::VectorXd>& endpoints);

  /**
   * @brief Factory method for clamped B-spline parameterization with uniform
   *   or geometrically spaced active knots.
   * @param horizon_steps Number of discrete time steps in the horizon.
   * @param degree Degree of B-spline polynomials. Must be less than
   *   num_control_points.
   * @param num_control_points Number of B-spline control points. Cannot be
   *   greater than horizon_steps.
   * @param knot_ratio Ratio of consecutive knot interval lengths (positive):
   *   the intervals are proportional to 1, knot_ratio, knot_ratio^2, ... and
   *   span [0, horizon_steps - 1]. The default of 1 gives uniform knots. A
   *   ratio above 1 puts short intervals (more flexibility) at the start of
   *   the horizon and a ratio below 1 puts them at the end.
   * @throws std::invalid_argument if the knot intervals are so short that the
   *   sampled basis does not have full column rank, or if any argument is
   *   invalid.
   * @return Parameterization instance.
   */
  static Parameterization bspline(int horizon_steps,
                                  int degree,
                                  int num_control_points,
                                  double knot_ratio = 1.0);

  /**
   * @brief Factory method for clamped B-spline parameterization with custom
   *   active knots.
   * @param horizon_steps Number of discrete time steps in the horizon.
   * @param degree Degree of B-spline polynomials. Must be less than
   *   horizon_steps.
   * @param active_knots Vector of active knots (need at least deg+1).
   * @return Parameterization instance.
   */
  static Parameterization
  bspline(int horizon_steps,
          int degree,
          const Eigen::Ref<const Eigen::VectorXd>& active_knots);

  /**
   * @brief Factory method for a custom basis given by its samples, e.g.
   *   Laguerre functions. The input at step k is
   *   `sum_i basis(k, i) * control_point_i`, where the control points are the
   *   basis coefficients.
   *
   *   A custom basis is treated as dense (every coefficient can affect every
   *   sample: band_width = num_control_points, band_start = 0, band_weights =
   *   basis transposed), and type is Type::Custom, with degree = -1 and empty
   *   knots. Since the B-spline convex hull property does not hold, an MPC
   *   with a custom basis always bounds every sampled input (whether or not
   *   `saturate_input_trajectory` is set), rejects `slew_control_points`, and
   *   rejects setReferenceInput(); the input cost weights the coefficients.
   * @param horizon_steps Number of discrete time steps in the horizon.
   * @param basis Basis sampled at k = 0, ..., horizon_steps-1, of size
   *   horizon_steps x num_control_points (one column per coefficient), with
   *   finite entries and full column rank: its smallest singular value must
   *   exceed 1e-9 times its largest. Full rank does not make the QP well
   *   conditioned; orthonormalizing the columns (e.g. by a QR decomposition)
   *   helps for bases such as slowly decaying Laguerre functions.
   * @throws std::invalid_argument if the shape is wrong, an entry is not
   *   finite, or the basis does not have full column rank.
   * @return Parameterization instance.
   */
  static Parameterization
  fromBasis(int horizon_steps, const Eigen::Ref<const Eigen::MatrixXd>& basis);

  /**
   * @brief Direct constructor for uniform clamped B-spline parameterization.
   * @param horizon_steps Number of discrete time steps in the horizon.
   * @param degree Degree of B-spline polynomials. Must be less than
   *   num_control_points.
   * @param num_control_points Number of B-spline control points. Cannot be
   *   greater than horizon_steps.
   *
   * For common cases, prefer the named factory methods.
   */
  Parameterization(int horizon_steps, int degree, int num_control_points);

  /**
   * @brief Direct constructor for advanced use cases with custom knot vector
   *   (e.g. unclamped B-splines).
   * @param horizon_steps Number of discrete time steps in the horizon.
   * @param degree Degree of B-spline polynomials. Must be less than
   *   horizon_steps.
   * @param knots Full knot vector with size in the range [2*(degree+1),
   *   horizon_steps+degree+1]. Must be non-decreasing. The first active knot
   *   (index degree) must be 0 and the last active knot (index
   *   knots.size()-degree-1) must be horizon_steps-1; for unclamped splines
   *   the knots outside them extend beyond [0, horizon_steps-1]. Knots among
   *   the first and last degree+1 are snapped to 0 and horizon_steps-1 when
   *   within `1e-9 * horizon_steps` (round-off).
   * @throws std::invalid_argument if the knots are invalid, including when the
   *   basis sampled at k = 0, ..., horizon_steps-1 does not have full column
   *   rank, so some combination of control points has no effect on any
   *   sampled input. By the Schoenberg-Whitney theorem this is the case unless
   *   every control point can be given its own sample, in increasing order, at
   *   which its basis function is nonzero. Repeated knots are allowed (each
   *   repeat lowers the continuity at that knot by one; multiplicity degree+1
   *   lets the input jump), except an interior knot equal to the first active
   *   knot or, for degree > 0, the last.
   */
  Parameterization(int horizon_steps,
                   int degree,
                   const Eigen::Ref<const Eigen::VectorXd>& knots);

  ~Parameterization() = default;

  /**
   * @brief Evaluate an input trajectory from provided control_points.
   *
   *   Uses the stored banded basis (see band_weights), so it only touches the
   *   control points that affect each step.
   * @param input_dim Dimension of input vector.
   * @param control_points Parameterized input trajectory as a vector of size
   *   input_dim*num_control_points.
   * @return input_traj The evaluated input trajectory.
   */
  Eigen::VectorXd
  evaluate(int input_dim,
           const Eigen::Ref<const Eigen::VectorXd>& control_points) const;

  /**
   * @brief Static method for generating a clamped knot vector with uniform or
   *   geometrically spaced active knots (the knots used by the factories).
   *   Useful for generating a starting point that can be modified for custom
   *   parameterizations.
   * @param horizon_steps Number of discrete time steps in the horizon.
   * @param degree Degree of B-spline polynomials. Must be less than
   *   num_control_points.
   * @param num_control_points Number of B-spline control points. Cannot be
   *   greater than horizon_steps.
   * @param knot_ratio Ratio of consecutive knot interval lengths (positive):
   *   the intervals are proportional to 1, knot_ratio, knot_ratio^2, ... and
   *   span [0, horizon_steps - 1]. The default of 1 gives uniform knots. A
   *   ratio above 1 puts short intervals (more flexibility) at the start of
   *   the horizon and a ratio below 1 puts them at the end.
   * @throws std::invalid_argument if the knot intervals are so short that the
   *   sampled basis does not have full column rank, or if any argument is
   *   invalid.
   * @return knots The clamped knot vector.
   */
  static Eigen::VectorXd makeClampedKnots(int horizon_steps,
                                          int degree,
                                          int num_control_points,
                                          double knot_ratio = 1.0);

  /**
   * @brief Static method for generating a uniform clamped knot vector. Useful
   *   for generating a starting point that can be modified for custom
   *   parameterizations.
   * @param horizon_steps Number of discrete time steps in the horizon.
   * @param degree Degree of B-spline polynomials. Must be less than
   *   num_control_points.
   * @param num_control_points Number of B-spline control points. Cannot be
   *   greater than horizon_steps.
   * @return knots The uniform clamped knot vector.
   */
  static Eigen::VectorXd makeUniformClampedKnots(int horizon_steps,
                                                 int degree,
                                                 int num_control_points);

  /// Number of discrete time steps in the horizon.
  const int horizon_steps;

  /// B-spline polynomial degree (-1 for a custom basis).
  const int degree;

  /// Number of B-spline control points (basis coefficients for a custom
  /// basis).
  const int num_control_points;

  /// Full knot vector of size num_control_points + degree + 1 (empty for a
  /// custom basis).
  const Eigen::VectorXd knots;

  /// Number of consecutive control points that can affect one sampled input
  /// (degree + 1 for B-splines, num_control_points for a custom basis).
  const int band_width;

  /// Index of the first control point that affects the input at each step k
  /// (size horizon_steps).
  const Eigen::VectorXi band_start;

  /// Basis weights in banded form (size band_width x horizon_steps). The input
  /// at step k is the sum over i of `band_weights(i, k)` times control point
  /// `band_start(k) + i`.
  const Eigen::MatrixXd band_weights;

  /// Kind of basis.
  const Type type;

private:
  friend class detail::VectorParameterization;

  // custom basis, validated by fromBasis()
  Parameterization(int horizon_steps, const Eigen::MatrixXd& basis);

  // Sample indices whose bounds are equivalent to bounding every sampled
  // input, or empty when bounding the control points is already exact. Not
  // public until the interface for input bound rows is settled.
  std::vector<int> inputSaturationSamples() const;
};

/**
 * @brief Stream a human-readable one-line summary of the parameterization.
 *
 * Long knot vectors are abbreviated in the middle with `...`.
 * @param os Output stream.
 * @param param Parameterization to print.
 * @return Reference to the output stream.
 */
std::ostream& operator<<(std::ostream& os, const Parameterization& param);

} // namespace affine_mpc

#endif // AFFINE_MPC_PARAMETERIZATION_HPP
