#ifndef AFFINE_MPC_DETAIL_VECTOR_PARAMETERIZATION_HPP
#define AFFINE_MPC_DETAIL_VECTOR_PARAMETERIZATION_HPP

/**
 * @file detail/vector_parameterization.hpp
 * @brief Defines VectorParameterization, the internal map from the control
 *   point vector of the QP to the inputs of every input channel.
 *
 * Internal: installed only because public headers include it (MPC and
 * MPCLogger hold it by value); not part of the public API.
 */

#include <Eigen/Core>
#include <vector>

#include "affine_mpc/parameterization.hpp"

namespace affine_mpc::detail {

/**
 * @class VectorParameterization
 * @brief Internal: maps the stacked control point vector to the inputs of all
 *   input channels. Used by the MPC classes; not part of the stable API and
 *   not exposed in Python.
 *
 * Coefficient i of channel j is element `offset(j) + stride(j) * i` of the
 * control point vector. Every channel currently shares one Parameterization
 * with the interleaved layout `offset(j) = j`, `stride(j) = input_dim`, so the
 * input_dim coefficients of one control point are consecutive and QP blocks
 * for one control point span input_dim columns starting at
 * `coefficientIndex(0, i)`.
 */
class VectorParameterization
{
public:
  /**
   * @brief Share one parameterization between all input channels.
   * @param input_dim Number of input channels (positive).
   * @param param Parameterization used by every channel.
   */
  VectorParameterization(int input_dim, const Parameterization& param);

  /// @brief The parameterization shared by every channel.
  const Parameterization& parameterization() const noexcept { return param_; }

  /// @brief Number of input channels.
  int inputDim() const noexcept { return input_dim_; }

  /// @brief Size of the control point vector.
  int numCoefficients() const noexcept { return num_coefficients_; }

  /// @brief Index in the control point vector of coefficient i of a channel.
  int coefficientIndex(int channel, int control_point) const noexcept
  {
    return offsets_(channel) + strides_(channel) * control_point;
  }

  /**
   * @brief Evaluate the input at one step.
   * @param k Step in [0, horizon_steps).
   * @param coefficients Control point vector of size numCoefficients().
   * @param uk Output input vector of size inputDim().
   */
  void evaluate(int k,
                const Eigen::Ref<const Eigen::VectorXd>& coefficients,
                Eigen::Ref<Eigen::VectorXd> uk) const noexcept;

  /**
   * @brief Evaluate the input trajectory.
   * @param coefficients Control point vector of size numCoefficients().
   * @param u_traj Output stacked inputs of size inputDim() * horizon_steps.
   */
  void evaluateTrajectory(const Eigen::Ref<const Eigen::VectorXd>& coefficients,
                          Eigen::Ref<Eigen::VectorXd> u_traj) const noexcept;

  /**
   * @brief Sample indices whose bounds are equivalent to bounding every
   *   sampled input (see Parameterization::inputSaturationSamples()).
   * @return Sorted sample indices, or empty when bounding the control points
   *   is already exact.
   */
  std::vector<int> inputSaturationSamples() const;

  /// The knot refinement of InputBoundMode::Refined (see refineKnots()).
  struct KnotRefinement
  {
    /// Refined knot vector (the knots plus the inserted ones).
    Eigen::VectorXd knots;
    /// Banded map R from the control points to the refined control points:
    /// refined control point j is the sum over i of `weights(i, j)` times
    /// control point `start(j) + i` (size degree+1 x num_refined).
    Eigen::VectorXi start;
    Eigen::MatrixXd weights;
    /// Refined control points whose basis function is nonzero at some sample
    /// (sorted); bounding them bounds every sampled input.
    std::vector<int> sampled;
  };

  /**
   * @brief Refine a B-spline by knot insertion (Oslo algorithm): insert knots
   *   until no active knot interval is longer than max_interval samples.
   * @param max_interval Longest refined knot interval in samples (positive).
   * @return The refined knots, the banded map R, and the refined control
   *   points the samples depend on.
   */
  KnotRefinement refineKnots(int max_interval) const;

private:
  const Parameterization param_;
  const int input_dim_;
  const int num_coefficients_;
  Eigen::VectorXi offsets_, strides_;
};

} // namespace affine_mpc::detail

#endif // AFFINE_MPC_DETAIL_VECTOR_PARAMETERIZATION_HPP
