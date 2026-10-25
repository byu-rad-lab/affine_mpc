#include "affine_mpc/discretizer.hpp"

#include <Eigen/Core>
#include <cmath>
#include <stdexcept>
#include <string>

using namespace Eigen;

namespace affine_mpc {

namespace { // utilities for this file

int validateDim(int dim, const char* name)
{
  if (dim <= 0)
    throw std::invalid_argument(std::string{"[Discretizer::Discretizer] "}
                                + name + " must be positive.");
  return dim;
}

void validateShape(Index rows,
                   Index cols,
                   Index expected_rows,
                   Index expected_cols,
                   const char* func,
                   const char* name)
{
  if (rows != expected_rows || cols != expected_cols)
    throw std::invalid_argument(
        std::string{"[Discretizer::"} + func + "] " + name + " must be "
        + std::to_string(expected_rows) + "x" + std::to_string(expected_cols)
        + ", got " + std::to_string(rows) + "x" + std::to_string(cols) + ".");
}

void validateSize(Index size,
                  Index expected,
                  const char* func,
                  const char* name)
{
  if (size != expected)
    throw std::invalid_argument(
        std::string{"[Discretizer::"} + func + "] " + name + " must have size "
        + std::to_string(expected) + ", got " + std::to_string(size) + ".");
}

} // namespace

Discretizer::Discretizer(int state_dim, int input_dim) :
    state_dim_{validateDim(state_dim, "state_dim")},
    input_dim_{validateDim(input_dim, "input_dim")},
    has_discretized_{false},
    E_{state_dim, state_dim},
    G_{state_dim, state_dim},
    At_{state_dim, state_dim},
    At_pow_{state_dim, state_dim},
    tmp_{state_dim, state_dim},
    Bd_tmp_{state_dim, input_dim},
    wd_tmp_{state_dim}
{}

void Discretizer::discretize(const Ref<const MatrixXd>& Ac,
                             const Ref<const MatrixXd>& Bc,
                             const Ref<const VectorXd>& wc,
                             double dt,
                             Ref<MatrixXd> Ad,
                             Ref<MatrixXd> Bd,
                             Ref<VectorXd> wd,
                             double tol)
{
  const int n{state_dim_}, m{input_dim_};
  validateShape(Ac.rows(), Ac.cols(), n, n, "discretize", "Ac");
  validateShape(Bc.rows(), Bc.cols(), n, m, "discretize", "Bc");
  validateSize(wc.size(), n, "discretize", "wc");
  validateShape(Ad.rows(), Ad.cols(), n, n, "discretize", "Ad");
  validateShape(Bd.rows(), Bd.cols(), n, m, "discretize", "Bd");
  validateSize(wd.size(), n, "discretize", "wd");
  if (!(dt > 0.0))
    throw std::invalid_argument(
        "[Discretizer::discretize] dt must be positive.");
  if (!(tol > 0.0))
    throw std::invalid_argument(
        "[Discretizer::discretize] tol must be positive.");

  // Computes E(dt) = exp(A*dt) and G(dt) = integral_0^dt exp(A*s) ds via
  // scaling and squaring. The Taylor series is only evaluated at h = dt / 2^s
  // where ||A*h||_1 <= 0.5, which keeps terms small and avoids the
  // cancellation that ruins the series for large ||A*dt||. Then
  // E(2h) = E(h)^2 and G(2h) = G(h) + E(h) G(h) recover dt.
  At_.noalias() = Ac * dt;
  const double At_norm{At_.cwiseAbs().colwise().sum().maxCoeff()};
  if (!std::isfinite(At_norm))
    throw std::invalid_argument(
        "[Discretizer::discretize] Ac*dt must be finite.");

  constexpr double max_norm{0.5};
  const int num_squarings{
      At_norm > max_norm
          ? static_cast<int>(std::ceil(std::log2(At_norm / max_norm)))
          : 0};
  const double scale{std::ldexp(1.0, -num_squarings)};
  At_ *= scale;

  At_pow_.setIdentity();
  E_.setIdentity();
  G_.setIdentity();

  // term_bound is an upper bound on ||(A*h)^i / i!||, the next term to add
  double factorial{1.0};
  double term_bound{At_norm * scale};
  for (int i{1}; term_bound >= tol; ++i) {
    tmp_.noalias() = At_pow_ * At_;
    At_pow_.swap(tmp_);
    factorial *= i;
    E_ += At_pow_ / factorial;
    G_ += At_pow_ / (factorial * (i + 1));
    term_bound *= At_norm * scale / (i + 1);
  }
  G_ *= dt * scale;

  for (int j{0}; j < num_squarings; ++j) {
    tmp_.noalias() = E_ * G_;
    G_ += tmp_; // G(2h) = G(h) + E(h) G(h), uses E(h) before squaring
    tmp_.noalias() = E_ * E_;
    E_.swap(tmp_); // E(2h) = E(h)^2
  }

  // through workspaces, so the outputs may alias the inputs
  Bd_tmp_.noalias() = G_ * Bc;
  wd_tmp_.noalias() = G_ * wc;
  Ad = E_;
  Bd = Bd_tmp_;
  wd = wd_tmp_;
  has_discretized_ = true;
}

void Discretizer::discretizeOffset(const Ref<const VectorXd>& wc,
                                   Ref<VectorXd> wd)
{
  validateSize(wc.size(), state_dim_, "discretizeOffset", "wc");
  validateSize(wd.size(), state_dim_, "discretizeOffset", "wd");
  if (!has_discretized_)
    throw std::logic_error("[Discretizer::discretizeOffset] discretize() must "
                           "be called first.");
  wd_tmp_.noalias() = G_ * wc;
  wd = wd_tmp_;
}

const MatrixXd& Discretizer::getExponentialIntegral() const
{
  if (!has_discretized_)
    throw std::logic_error("[Discretizer::getExponentialIntegral] "
                           "discretize() must be called first.");
  return G_;
}

} // namespace affine_mpc
