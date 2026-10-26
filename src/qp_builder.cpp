#include "qp_builder.hpp"

#include <Eigen/Core>
#include <algorithm>
#include <cstddef>
#include <memory>
#include <utility>
#include <vector>

#include "affine_mpc/options.hpp"
#include "affine_mpc/parameterization.hpp"
#include "formulation.hpp"
#include "qp_terms.hpp"

using namespace Eigen;

namespace affine_mpc::detail {

namespace {

std::unique_ptr<Formulation>
makeFormulation(const FormulationKind kind,
                const int state_dim,
                const VectorParameterization& input_param,
                QP& qp)
{
  if (kind == FormulationKind::Sparse)
    return std::make_unique<SparseFormulation>(state_dim, input_param, qp);
  return std::make_unique<CondensedFormulation>(state_dim, input_param, qp);
}

// Allocate M (rows x cols, CSC) with the declared entries as its pattern,
// each entry once and with zero values. With `upper`, M is the upper triangle
// of a symmetric matrix: entries below the diagonal are left out and the whole
// diagonal is stored (OSQP keeps it in any case).
void allocatePattern(QP::SparseMatrix& M,
                     const Index rows,
                     const Index cols,
                     const std::vector<QPEntries*>& sets,
                     const bool upper)
{
  // the entries as col * rows + row, sorted in CSC order
  std::size_t count{upper ? static_cast<std::size_t>(cols) : 0};
  for (const QPEntries* entries : sets)
    count += static_cast<std::size_t>(entries->size());
  std::vector<Index> keys;
  keys.reserve(count);
  if (upper)
    for (Index j{0}; j < cols; ++j)
      keys.push_back(j * rows + j);
  for (const QPEntries* entries : sets)
    for (Index i{0}; i < entries->size(); ++i)
      if (!upper || entries->row(i) <= entries->col(i))
        keys.push_back(entries->col(i) * rows + entries->row(i));
  std::sort(keys.begin(), keys.end());
  keys.erase(std::unique(keys.begin(), keys.end()), keys.end());

  M.resize(rows, cols);
  M.resizeNonZeros(static_cast<Index>(keys.size()));
  auto* outer{M.outerIndexPtr()};
  auto* inner{M.innerIndexPtr()};
  std::fill(outer, outer + cols + 1, 0);
  for (std::size_t k{0}; k < keys.size(); ++k) {
    ++outer[keys[k] / rows + 1];
    inner[k] = static_cast<OSQPInt>(keys[k] % rows);
  }
  for (Index j{0}; j < cols; ++j)
    outer[j + 1] += outer[j];
  M.coeffs().setZero();
}

} // namespace

template <class Term, class... Args>
const Term& QPBuilder::add(QPLayout& layout, Args&&... args)
{
  auto term{std::make_unique<Term>(std::forward<Args>(args)...)};
  term->declare(layout, *formulation_);
  const Term& ref{*term};
  terms_.push_back(std::move(term));
  return ref;
}

QPBuilder::QPBuilder(const FormulationKind kind,
                     const int state_dim,
                     const VectorParameterization& input_param,
                     const Options& opts) :
    formulation_{makeFormulation(kind, state_dim, input_param, qp_)}
{
  // The options select the terms; each term's rows follow the previous ones
  QPLayout layout{formulation_->numVariables()};
  rows_.dynamics = formulation_->declare(layout);
  add<StateCost>(layout);
  // before InputCost, so the sparse formulation assigns its control point
  // block once instead of zeroing it first for InputCost's diagonal
  if (opts.use_input_rate_cost)
    add<InputRateCost>(layout, input_param);
  if (opts.use_input_cost)
    add<InputCost>(layout);
  rows_.input_bounds =
      add<InputBounds>(layout, input_param, opts.saturate_input_trajectory)
          .rows();
  if (opts.slew_initial_input)
    rows_.initial_slew = add<InitialSlew>(layout, input_param).rows();
  if (opts.slew_control_points)
    rows_.control_point_slew =
        add<ControlPointSlew>(layout, input_param).rows();
  if (opts.saturate_states) {
    const StateBounds& bounds{add<StateBounds>(
        layout, boundedStates(opts, state_dim), state_dim,
        input_param.parameterization().horizon_steps, opts.soft_state_bounds)};
    rows_.state_bounds = bounds.rows();
    if (opts.soft_state_bounds)
      add<SoftStatePenalty>(layout, bounds.slack());
  }
  if (opts.num_general_constraints > 0)
    rows_.general_constraints =
        add<GeneralConstraints>(layout, input_param,
                                opts.num_general_constraints, state_dim)
            .rows();
  if (opts.num_terminal_constraints > 0)
    rows_.terminal_constraints =
        add<TerminalConstraints>(layout, opts.num_terminal_constraints,
                                 state_dim,
                                 input_param.parameterization().horizon_steps)
            .rows();
  num_constraints_ = layout.numRows();
  num_variables_ = layout.numVariables();
  // variables the terms declared (after the formulation's own)
  const int own_variables{formulation_->numVariables()};
  formulation_->declareExtraVariables(
      {own_variables, num_variables_ - own_variables}, layout);
  declared_P_ = layout.entriesP();
  declared_A_ = layout.entriesA();

  // Allocate the QP in the formulation's storage; the declared entries'
  // places depend on it and on the final sizes
  qp_.q.setZero(num_variables_);
  qp_.l.setZero(num_constraints_);
  qp_.u.setZero(num_constraints_);
  if (kind == FormulationKind::Sparse) {
    // CSC with the declared entries as the pattern (every write is to a
    // declared entry)
    qp_.storage = QPStorage::Sparse;
    allocatePattern(qp_.P_csc, num_variables_, num_variables_, declared_P_,
                    true);
    allocatePattern(qp_.A_csc, num_constraints_, num_variables_, declared_A_,
                    false);
    for (QPEntries* entries : declared_P_)
      entries->resolve(qp_.P_csc, true);
    for (QPEntries* entries : declared_A_)
      entries->resolve(qp_.A_csc, false);
  } else {
    // dense: the condensed formulation also writes the matrices directly
    qp_.storage = QPStorage::Dense;
    qp_.P.setZero(num_variables_, num_variables_);
    qp_.A.setZero(num_constraints_, num_variables_);
    for (QPEntries* entries : declared_P_)
      entries->resolve(qp_.P);
    for (QPEntries* entries : declared_A_)
      entries->resolve(qp_.A);
  }

  // Sort the term parts by when they are updated
  bounds_x0_ = formulation_->updateX0WritesBounds();
  for (const auto& term : terms_) {
    const DataChangeSet matrix_deps{term->matrixDependsOn(*formulation_)};
    const DataChangeSet vector_deps{term->vectorDependsOn(*formulation_)};
    if (term->isCost()) {
      cost_terms_.push_back(term.get());
      cost_matrix_deps_ |= matrix_deps;
      cost_vector_deps_ |= vector_deps;
      continue;
    }
    if (matrix_deps.empty())
      constant_row_matrices_.push_back(term.get());
    else
      row_matrices_.push_back({term.get(), matrix_deps});
    if (vector_deps.has(DataChange::X0)) {
      row_vectors_x0_.push_back(term.get());
      bounds_x0_ = true;
    } else {
      row_vectors_.push_back({term.get(), vector_deps});
    }
  }
  cost_vector_x0_ = cost_vector_deps_.has(DataChange::X0);
}

const MatrixXd& QPBuilder::denseCostMatrix() const
{
  if (qp_.storage == QPStorage::Dense)
    return qp_.P;
  // allocates on the first call only (same size afterwards)
  detail::denseCostMatrix(qp_, dense_P_);
  return dense_P_;
}

void QPBuilder::writeConstantParts(const MPCData& d)
{
  formulation_->writeConstantRows();
  for (QPTerm* term : constant_row_matrices_)
    term->updateMatrix(d, *formulation_, qp_);
}

Formulation::Update QPBuilder::apply(const DataChangeSet changes,
                                     const MPCData& d)
{
  Formulation& f{*formulation_};
  // first the formulation (the terms read its state map), then every matrix
  // part before any vector part (a vector part may read what its matrix part
  // cached, see QPTerm::vectorDependsOn())
  Formulation::Update written{f.update(changes, d)};

  if (changes.has(cost_matrix_deps_)) {
    f.beginCostMatrix();
    for (QPTerm* term : cost_terms_)
      term->updateMatrix(d, f, qp_);
    f.endCostMatrix();
    written.P |= changes & cost_matrix_deps_;
  }
  for (const Entry& e : row_matrices_) {
    if (changes.has(e.deps)) {
      e.term->updateMatrix(d, f, qp_);
      written.A |= changes & e.deps;
    }
  }

  if (!cost_vector_x0_ && changes.has(cost_vector_deps_)) {
    f.beginCostVector();
    for (QPTerm* term : cost_terms_)
      term->updateVector(d, f, qp_);
    f.endCostVector();
    written.q |= changes & cost_vector_deps_;
  }
  for (const Entry& e : row_vectors_) {
    if (changes.has(e.deps)) {
      e.term->updateVector(d, f, qp_);
      written.bounds |= changes & e.deps;
    }
  }
  return written;
}

void QPBuilder::applyX0(const MPCData& d)
{
  Formulation& f{*formulation_};
  f.updateX0(d);
  if (cost_vector_x0_) {
    f.beginCostVector();
    for (QPTerm* term : cost_terms_)
      term->updateVector(d, f, qp_);
    f.endCostVector();
  }
  for (QPTerm* term : row_vectors_x0_)
    term->updateVector(d, f, qp_);
}

} // namespace affine_mpc::detail
