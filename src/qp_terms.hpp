#ifndef AFFINE_MPC_QP_TERMS_HPP
#define AFFINE_MPC_QP_TERMS_HPP

// Internal (not installed): the features of the MPC problem as QP terms. Each
// term writes one cost or one block of constraint rows through the
// formulation, so the same term works for every formulation.

#include <Eigen/Core>
#include <vector>

#include "affine_mpc/detail/mpc_data.hpp"
#include "affine_mpc/detail/vector_parameterization.hpp"
#include "formulation.hpp"

namespace affine_mpc::detail {

/**
 * One feature of the QP: a cost (added into P and q through the formulation)
 * or a block of constraint rows it owns and overwrites.
 *
 * The matrix part (P or the rows of A) and the vector part (q or the bounds)
 * are updated separately, each when a category it depends on changes. Parts
 * that depend on X0 are updated by every solve(); the others by prepare() or
 * the start of solve(). Parts with no dependencies are written once.
 */
class QPTerm
{
public:
  virtual ~QPTerm() = default;

  /// Declare the constraint rows the term owns (costs own none).
  virtual void declare(QPLayout& /*layout*/) {}
  /// Whether the term is a cost (adds into P and q).
  virtual bool isCost() const noexcept { return false; }
  /// Categories that change the term's part of P or of A.
  virtual DataChangeSet matrixDependsOn(const Formulation& f) const = 0;
  /// Categories that change the term's part of q or of the bounds.
  virtual DataChangeSet vectorDependsOn(const Formulation& f) const = 0;
  virtual void updateMatrix(const MPCData& d, Formulation& f, QPRef& qp) = 0;
  virtual void updateVector(const MPCData& d, Formulation& f, QPRef& qp) = 0;

  /// The rows declared by declare() (empty for costs).
  const RowBlock& rows() const noexcept { return rows_; }

protected:
  RowBlock rows_{};
};

/// 1/2 sum_k (x_k - x_ref,k)^T Q_k (x_k - x_ref,k), with Q_{T-1} = Q_f.
class StateCost final : public QPTerm
{
public:
  bool isCost() const noexcept override { return true; }
  DataChangeSet matrixDependsOn(const Formulation& f) const override
  {
    return DataChangeSet{DataChange::Weights} | f.stateMatrixDependsOn();
  }
  DataChangeSet vectorDependsOn(const Formulation& f) const override
  {
    return DataChange::Weights | DataChange::References
           | f.stateMatrixDependsOn() | f.stateOffsetDependsOn();
  }
  void updateMatrix(const MPCData& d, Formulation& f, QPRef& qp) override;
  void updateVector(const MPCData& d, Formulation& f, QPRef& qp) override;
};

/// 1/2 sum_i (c_i - c_ref,i)^T R (c_i - c_ref,i) on the control points
/// (`use_input_cost`).
class InputCost final : public QPTerm
{
public:
  bool isCost() const noexcept override { return true; }
  DataChangeSet matrixDependsOn(const Formulation&) const override
  {
    return DataChange::Weights;
  }
  DataChangeSet vectorDependsOn(const Formulation&) const override
  {
    return DataChange::Weights | DataChange::References;
  }
  void updateMatrix(const MPCData& d, Formulation& f, QPRef& qp) override;
  void updateVector(const MPCData& d, Formulation& f, QPRef& qp) override;
};

/// u_min <= input <= u_max, on the control points or, with
/// `saturate_input_trajectory`, on the sampled inputs that make it exact.
class InputBounds final : public QPTerm
{
public:
  InputBounds(const VectorParameterization& input_param,
              bool saturate_input_trajectory);
  void declare(QPLayout& layout) override;
  DataChangeSet matrixDependsOn(const Formulation&) const override
  {
    return {};
  }
  DataChangeSet vectorDependsOn(const Formulation&) const override
  {
    return DataChange::Limits;
  }
  void updateMatrix(const MPCData& d, Formulation& f, QPRef& qp) override;
  void updateVector(const MPCData& d, Formulation& f, QPRef& qp) override;

private:
  const VectorParameterization& input_param_;
  // bounded sample indices (empty: the control points are bounded instead)
  const std::vector<int> samples_;
  const int num_bounded_; // number of bounded inputs (samples or points)
};

/// |u_0 - u_prev| <= u0_slew (`slew_initial_input`).
class InitialSlew final : public QPTerm
{
public:
  explicit InitialSlew(const VectorParameterization& input_param) :
      input_param_{input_param}
  {}
  void declare(QPLayout& layout) override
  {
    rows_ = layout.addRows(input_param_.inputDim());
  }
  DataChangeSet matrixDependsOn(const Formulation&) const override
  {
    return {};
  }
  DataChangeSet vectorDependsOn(const Formulation&) const override
  {
    return DataChange::Limits | DataChange::PreviousInput;
  }
  void updateMatrix(const MPCData& d, Formulation& f, QPRef& qp) override;
  void updateVector(const MPCData& d, Formulation& f, QPRef& qp) override;

private:
  const VectorParameterization& input_param_;
};

/// |c_{i+1} - c_i| <= control_point_slew (`slew_control_points`).
class ControlPointSlew final : public QPTerm
{
public:
  explicit ControlPointSlew(const VectorParameterization& input_param) :
      input_param_{input_param}
  {}
  void declare(QPLayout& layout) override
  {
    rows_ = layout.addRows(input_param_.numCoefficients()
                           - input_param_.inputDim());
  }
  DataChangeSet matrixDependsOn(const Formulation&) const override
  {
    return {};
  }
  DataChangeSet vectorDependsOn(const Formulation&) const override
  {
    return DataChange::Limits;
  }
  void updateMatrix(const MPCData& d, Formulation& f, QPRef& qp) override;
  void updateVector(const MPCData& d, Formulation& f, QPRef& qp) override;

private:
  const VectorParameterization& input_param_;
};

/// x_min <= x_k <= x_max at every step (`saturate_states`).
class StateBounds final : public QPTerm
{
public:
  explicit StateBounds(int num_states) : num_states_{num_states} {}
  void declare(QPLayout& layout) override
  {
    rows_ = layout.addRows(num_states_);
  }
  DataChangeSet matrixDependsOn(const Formulation& f) const override
  {
    return f.stateMatrixDependsOn();
  }
  DataChangeSet vectorDependsOn(const Formulation& f) const override
  {
    return DataChangeSet{DataChange::Limits} | f.stateOffsetDependsOn();
  }
  void updateMatrix(const MPCData& d, Formulation& f, QPRef& qp) override;
  void updateVector(const MPCData& d, Formulation& f, QPRef& qp) override;

private:
  const int num_states_; // state_dim * horizon_steps
};

} // namespace affine_mpc::detail

#endif // AFFINE_MPC_QP_TERMS_HPP
