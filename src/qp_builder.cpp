#include "qp_builder.hpp"

#include <Eigen/Core>
#include <memory>
#include <utility>

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
                const QPRef qp)
{
  if (kind == FormulationKind::Sparse)
    return std::make_unique<SparseFormulation>(state_dim, input_param, qp);
  return std::make_unique<CondensedFormulation>(state_dim, input_param, qp);
}

} // namespace

template <class Term, class... Args>
const Term& QPBuilder::add(QPLayout& layout, Args&&... args)
{
  auto term{std::make_unique<Term>(std::forward<Args>(args)...)};
  term->declare(layout);
  const Term& ref{*term};
  terms_.push_back(std::move(term));
  return ref;
}

QPBuilder::QPBuilder(const FormulationKind kind,
                     const int state_dim,
                     const VectorParameterization& input_param,
                     const Options& opts,
                     const QPRef qp) :
    qp_{qp}, formulation_{makeFormulation(kind, state_dim, input_param, qp)}
{
  // The options select the terms; each term's rows follow the previous ones
  QPLayout layout;
  rows_.dynamics = formulation_->declareRows(layout);
  add<StateCost>(layout);
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
    const int num_states{state_dim
                         * input_param.parameterization().horizon_steps};
    rows_.state_bounds = add<StateBounds>(layout, num_states).rows();
  }
  num_constraints_ = layout.numRows();

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
  // first the formulation (the terms read its state map)
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

void QPBuilder::applyX0(const MPCData& d, const Ref<const VectorXd>& x0)
{
  Formulation& f{*formulation_};
  f.updateX0(d, x0);
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
