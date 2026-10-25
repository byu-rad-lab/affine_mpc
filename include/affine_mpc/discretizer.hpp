#ifndef AFFINE_MPC_DISCRETIZER_HPP
#define AFFINE_MPC_DISCRETIZER_HPP

/**
 * @file discretizer.hpp
 * @brief Defines the Discretizer class for exact zero-order-hold
 *   discretization of continuous-time affine models.
 */

#include <Eigen/Core>

namespace affine_mpc {

/**
 * @class Discretizer
 * @brief Discretizes a continuous-time affine model `dx/dt = Ac x + Bc u + wc`
 *   into `x_next = Ad x + Bd u + wd` assuming the input is held constant over
 *   each time step (zero-order hold).
 *
 * The result is exact up to the truncation tolerance:
 * `Ad = exp(Ac dt)`, `Bd = G Bc`, and `wd = G wc` with
 * `G = integral_0^dt exp(Ac s) ds`. The matrix exponential is computed with
 * scaling and squaring: the step is halved `s` times until `||Ac h||_1 <= 0.5`
 * (with `h = dt / 2^s`), a Taylor series is evaluated at `h`, and the result is
 * squared `s` times to recover `dt`. The Taylor series stops once the bound on
 * the next term, `||Ac h||_1^i / i!`, is smaller than `tol`.
 *
 * Working memory is allocated at construction, so discretizing does not
 * allocate. The last `G` is kept, so a new model offset can be discretized
 * without recomputing the matrix exponential (see discretizeOffset()). Useful
 * on its own for simulation and for updating the affine approximation of a
 * nonlinear model.
 */
class Discretizer
{
public:
  /**
   * @brief Construct a Discretizer for models of the given dimensions.
   * @param state_dim State vector dimension (positive).
   * @param input_dim Input vector dimension (positive).
   * @throws std::invalid_argument if a dimension is not positive.
   */
  Discretizer(int state_dim, int input_dim);

  /**
   * @brief Discretize a continuous-time affine model.
   *
   *   The outputs may alias the inputs (e.g. `Ad` and `Ac` may be the same
   *   matrix).
   * @param Ac Continuous-time state matrix (state_dim x state_dim).
   * @param Bc Continuous-time input matrix (state_dim x input_dim).
   * @param wc Continuous-time model offset (size state_dim).
   * @param dt Time step in seconds (positive).
   * @param Ad Output discrete-time state matrix (state_dim x state_dim).
   * @param Bd Output discrete-time input matrix (state_dim x input_dim).
   * @param wd Output discrete-time model offset (size state_dim).
   * @param tol Truncation tolerance for the Taylor series (positive). The
   *   actual error is typically two to three orders of magnitude smaller than
   *   `tol`. The default gives results close to machine precision; larger
   *   values (e.g., 1e-6) save a few matrix products.
   * @throws std::invalid_argument if a size is wrong, `dt` or `tol` is not
   *   positive, or `Ac * dt` is not finite.
   */
  void discretize(const Eigen::Ref<const Eigen::MatrixXd>& Ac,
                  const Eigen::Ref<const Eigen::MatrixXd>& Bc,
                  const Eigen::Ref<const Eigen::VectorXd>& wc,
                  double dt,
                  Eigen::Ref<Eigen::MatrixXd> Ad,
                  Eigen::Ref<Eigen::MatrixXd> Bd,
                  Eigen::Ref<Eigen::VectorXd> wd,
                  double tol = 1e-10);

  /**
   * @brief Discretize only a model offset with the state matrix and time step
   *   of the last discretize() call: `wd = G wc`.
   *
   *   Costs one matrix-vector product, with no matrix exponential.
   * @param wc Continuous-time model offset (size state_dim).
   * @param wd Output discrete-time model offset (size state_dim). May alias
   *   `wc`.
   * @throws std::invalid_argument if a size is wrong.
   * @throws std::logic_error if discretize() has not been called.
   */
  void discretizeOffset(const Eigen::Ref<const Eigen::VectorXd>& wc,
                        Eigen::Ref<Eigen::VectorXd> wd);

  /// @brief Whether discretize() has been called.
  bool hasDiscretized() const noexcept { return has_discretized_; }

  /**
   * @brief Get `G = integral_0^dt exp(Ac s) ds` from the last discretize()
   *   call (`Bd = G Bc`, `wd = G wc`).
   * @throws std::logic_error if discretize() has not been called.
   */
  const Eigen::MatrixXd& getExponentialIntegral() const;

  /// @brief Get the state dimension.
  int getStateDim() const noexcept { return state_dim_; }

  /// @brief Get the input dimension.
  int getInputDim() const noexcept { return input_dim_; }

private:
  const int state_dim_, input_dim_;
  bool has_discretized_;

  // E = exp(Ac h) and G = integral_0^h exp(Ac s) ds while squaring up to dt
  Eigen::MatrixXd E_, G_;
  Eigen::MatrixXd At_, At_pow_, tmp_;
  Eigen::MatrixXd Bd_tmp_;
  Eigen::VectorXd wd_tmp_;
};

} // namespace affine_mpc

#endif // AFFINE_MPC_DISCRETIZER_HPP
