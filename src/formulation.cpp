#include "formulation.hpp"

#include <Eigen/Core>
#include <algorithm>
#include <cassert>
#include <cstddef>
#include <vector>

#include "affine_mpc/parameterization.hpp"

using namespace Eigen;

namespace affine_mpc::detail {

namespace {

// Assign or add `value` to a part written whole, then mark it written. Only
// for coefficient-wise values: a matrix product would be evaluated into a
// temporary (a heap allocation) without noalias().
template <class Part, class Dst, class Src>
void assignOrAdd(Part& part, Dst&& dst, const Src& value)
{
  if (part.stale)
    dst = value;
  else
    dst += value;
  part.stale = false;
  part.written = true;
}

// For each step, the number of leading control point columns that can affect
// the state after it (the columns of every control point up to the last one
// in the band of this or an earlier step)
std::vector<int> bandColumns(const VectorParameterization& input_param)
{
  const Parameterization& param{input_param.parameterization()};
  std::vector<int> cols(static_cast<std::size_t>(param.horizon_steps));
  int last{0};
  for (int k{0}; k < param.horizon_steps; ++k) {
    last = std::max(last, param.band_start(k) + param.band_width - 1);
    int c{0};
    for (int ch{0}; ch < input_param.inputDim(); ++ch)
      c = std::max(c, input_param.coefficientIndex(ch, last) + 1);
    cols[static_cast<std::size_t>(k)] = c;
  }
  return cols;
}

} // namespace

// ---- StateRowMap ------------------------------------------------------------

StateRowMap StateRowMap::selection(const std::vector<int>& states,
                                   const int state_dim)
{
  const int num_selected{static_cast<int>(states.size())};
  StateRowMap map{num_selected, num_selected == state_dim};
  map.entries_.reserve(states.size());
  for (int j{0}; j < num_selected; ++j)
    map.entries_.push_back({j, states[static_cast<std::size_t>(j)], 1.0});
  return map;
}

StateRowMap StateRowMap::general(const int rows, const int state_dim)
{
  StateRowMap map{rows, false};
  map.entries_.reserve(static_cast<std::size_t>(rows)
                       * static_cast<std::size_t>(state_dim));
  return map;
}

void StateRowMap::assign(const Ref<const MatrixXd>& H)
{
  assert(H.rows() == rows_);
  assert(static_cast<Index>(entries_.capacity()) >= H.size());
  entries_.clear();
  for (int i{0}; i < rows_; ++i)
    for (int j{0}; j < H.cols(); ++j)
      if (H(i, j) != 0.0)
        entries_.push_back({i, j, H(i, j)});
}

bool StateRowMap::matches(const Ref<const MatrixXd>& H) const
{
  if (H.rows() != rows_)
    return false;
  std::size_t next{0};
  for (int i{0}; i < rows_; ++i)
    for (int j{0}; j < H.cols(); ++j) {
      if (H(i, j) == 0.0)
        continue;
      if (next == entries_.size())
        return false;
      const Entry& e{entries_[next++]};
      if (e.row != i || e.state != j || e.value != H(i, j))
        return false;
    }
  return next == entries_.size();
}

// ---- CondensedFormulation ---------------------------------------------------

CondensedFormulation::CondensedFormulation(
    const int state_dim,
    const VectorParameterization& input_param,
    const QPRef qp) :
    Formulation{qp},
    n_{state_dim},
    m_{input_param.inputDim()},
    T_{input_param.parameterization().horizon_steps},
    nx_{state_dim * T_},
    nc_{input_param.numCoefficients()},
    input_param_{input_param},
    band_cols_{bandColumns(input_param)},
    // the first row block of S is only written in the band columns, so the
    // rest of it must stay zero
    S_{MatrixXd::Zero(nx_, nc_)},
    v_{VectorXd::Zero(nx_)},
    weighted_S_{nx_, nc_},
    weighted_state_err_{nx_}
{}

Formulation::Update CondensedFormulation::update(const DataChangeSet changes,
                                                 const MPCData& d)
{
  // S only feeds the terms, which write the QP
  if (changes.has(DataChange::Model))
    updateS(d);
  return {};
}

void CondensedFormulation::updateX0(const MPCData& d) { updateV(d); }

void CondensedFormulation::predictStates(const Ref<const VectorXd>& z,
                                         Ref<VectorXd> x_traj) const noexcept
{
  x_traj.noalias() = S_ * z.head(nc_);
  x_traj += v_;
}

void CondensedFormulation::addStateQuadraticMatrix(const Ref<const VectorXd>& Q)
{
  // P = S^T diag(Q) S as one product. Eigen's matrix product still
  // heap-allocates its blocking buffers when they exceed
  // EIGEN_STACK_ALLOCATION_LIMIT (128 KB), i.e. for large problems.
  // Products with a DiagonalMatrix copy it (a heap allocation) in Eigen 3.4,
  // so the weights are applied through asDiagonal() of a vector instead.
  weighted_S_.noalias() = Q.asDiagonal() * S_;
  if (P_.stale)
    ownP().noalias() = S_.transpose() * weighted_S_;
  else
    ownP().noalias() += S_.transpose() * weighted_S_;
  P_.stale = false;
  P_.written = true;
}

void CondensedFormulation::addStateQuadraticVector(const Ref<const VectorXd>& Q,
                                                   const Ref<const VectorXd>& r)
{
  // q = S^T diag(Q) (v - r)
  weighted_state_err_.noalias() = Q.cwiseProduct(v_ - r);
  if (q_.stale)
    ownQ().noalias() = S_.transpose() * weighted_state_err_;
  else
    ownQ().noalias() += S_.transpose() * weighted_state_err_;
  q_.stale = false;
  q_.written = true;
}

void CondensedFormulation::addControlQuadraticMatrix(
    const Ref<const VectorXd>& R)
{
  startPartialWrite(P_, ownP());
  ownP().diagonal() += R;
}

void CondensedFormulation::addControlQuadraticVector(
    const Ref<const VectorXd>& R, const Ref<const VectorXd>& c_ref)
{
  startPartialWrite(q_, ownQ());
  ownQ().noalias() -= R.cwiseProduct(c_ref);
}

void CondensedFormulation::addControlMatrix(const Ref<const MatrixXd>& P_c)
{
  startPartialWrite(P_, ownP());
  ownP() += P_c;
}

void CondensedFormulation::addControlLinear(const Ref<const VectorXd>& q_c)
{
  startPartialWrite(q_, ownQ());
  ownQ() += q_c;
}

void CondensedFormulation::setStateRows(const RowBlock& block,
                                        const StateRowMap& H,
                                        const int first_step,
                                        const int num_steps)
{
  if (H.isIdentity()) {
    // the rows of S itself
    qp_.A.block(block.offset, 0, num_steps * n_, nc_) =
        S_.middleRows((first_step - 1) * n_, num_steps * n_);
    return;
  }
  // Row i of H S_k (S_k: the rows of S for x_k) as a sum over the nonzero
  // entries of row i of H. A selection of states copies rows of S.
  const int p{H.rows()};
  const std::vector<StateRowMap::Entry>& entries{H.entries()};
  for (int k{first_step}, row{block.offset}; k < first_step + num_steps;
       ++k, row += p) {
    const int t{(k - 1) * n_};
    const int cols{band_cols_[static_cast<std::size_t>(k - 1)]};
    qp_.A.block(row, cols, p, nc_ - cols).setZero();
    auto e{entries.begin()};
    for (int i{0}; i < p; ++i) {
      auto dst{qp_.A.row(row + i).head(cols)};
      if (e == entries.end() || e->row != i) {
        dst.setZero();
        continue;
      }
      dst = e->value * S_.row(t + e->state).head(cols);
      for (++e; e != entries.end() && e->row == i; ++e)
        dst += e->value * S_.row(t + e->state).head(cols);
    }
  }
}

void CondensedFormulation::setStateBounds(const RowBlock& block,
                                          const StateRowMap& H,
                                          const int first_step,
                                          const int num_steps,
                                          const Ref<const VectorXd>& lo,
                                          const Ref<const VectorXd>& hi)
{
  // lo <= H (S_k c + v_k) <= hi becomes lo - H v_k <= H S_k c <= hi - H v_k
  if (H.isIdentity()) {
    const int rows{num_steps * n_};
    const auto v{v_.segment((first_step - 1) * n_, rows)};
    qp_.l.segment(block.offset, rows) = lo.replicate(num_steps, 1) - v;
    qp_.u.segment(block.offset, rows) = hi.replicate(num_steps, 1) - v;
    return;
  }
  const int p{H.rows()};
  const std::vector<StateRowMap::Entry>& entries{H.entries()};
  for (int k{first_step}, row{block.offset}; k < first_step + num_steps;
       ++k, row += p) {
    const int t{(k - 1) * n_};
    auto e{entries.begin()};
    for (int i{0}; i < p; ++i) {
      // (H v_k)_i, the first entry assigned (a selection copies v exactly)
      double offset{0.0};
      if (e != entries.end() && e->row == i) {
        offset = e->value * v_(t + e->state);
        for (++e; e != entries.end() && e->row == i; ++e)
          offset += e->value * v_(t + e->state);
      }
      qp_.l(row + i) = lo(i) - offset;
      qp_.u(row + i) = hi(i) - offset;
    }
  }
}

void CondensedFormulation::updateS(const MPCData& d)
{
  const Parameterization& param{input_param_.parameterization()};
  const MatrixXd& Ad{d.Ad};
  const MatrixXd& Bd{d.Bd};

  // add dynamics of initial time step (k=0)
  for (int i{0}; i < param.band_width; ++i) {
    const int col{input_param_.coefficientIndex(0, param.band_start(0) + i)};
    S_.block(0, col, n_, m_) = param.band_weights(i, 0) * Bd;
  }

  for (int k{1}, row{n_}; k < T_; ++k, row += n_) {
    // add cumulative effect of all previous dynamics
    S_.middleRows(row, n_).noalias() = Ad * S_.middleRows(row - n_, n_);

    // add effect of current input parameterized by the basis
    for (int i{0}; i < param.band_width; ++i) {
      const int col{input_param_.coefficientIndex(0, param.band_start(k) + i)};
      S_.block(row, col, n_, m_) += param.band_weights(i, k) * Bd;
    }
  }
}

void CondensedFormulation::updateV(const MPCData& d)
{
  // add dynamics of initial time step (k=0). Adding wd separately avoids a
  // temporary for the product inside a sum.
  v_.head(n_).noalias() = d.Ad * d.x0;
  v_.head(n_) += d.wd;

  for (int k{1}, row{n_}; k < T_; ++k, row += n_) {
    // add cumulative effect of all previous dynamics
    v_.segment(row, n_).noalias() = d.Ad * v_.segment(row - n_, n_);
    v_.segment(row, n_) += d.wd;
  }
}

// ---- SparseFormulation ------------------------------------------------------

SparseFormulation::SparseFormulation(const int state_dim,
                                     const VectorParameterization& input_param,
                                     const QPRef qp) :
    Formulation{qp},
    n_{state_dim},
    m_{input_param.inputDim()},
    T_{input_param.parameterization().horizon_steps},
    nx_{state_dim * T_},
    nc_{input_param.numCoefficients()},
    input_param_{input_param}
{}

void SparseFormulation::writeConstantRows()
{
  // -x_{k+1} in the dynamics rows (the declared rows are the first rows of A)
  qp_.A.block(0, nc_, nx_, nx_).diagonal().setConstant(-1.0);
}

Formulation::Update SparseFormulation::update(const DataChangeSet changes,
                                              const MPCData& d)
{
  Update written{};
  if (changes.has(DataChange::Model)) {
    setDynamicsMatrixBlocks(d);
    written.A = DataChange::Model;
  }
  if (changes.has(DataChange::ModelOffset)) {
    setDynamicsOffsetBounds(d);
    written.bounds = DataChange::ModelOffset;
  }
  return written;
}

void SparseFormulation::updateX0(const MPCData& d)
{
  // the k = 0 dynamics rows: -x_1 + Bd u_0 = -Ad x0 - wd. Separate statements
  // avoid a temporary for the product inside a sum.
  qp_.l.head(n_) = -d.wd;
  qp_.l.head(n_).noalias() -= d.Ad * d.x0;
  qp_.u.head(n_) = qp_.l.head(n_);
}

void SparseFormulation::writeCostPlaceholders()
{
  // Store every weight position on P's diagonal so a weight that is zero now
  // can become nonzero later. This costs nothing in solves since OSQP's KKT
  // system always includes P's diagonal. The weights are re-applied after
  // initialization.
  qp_.P.diagonal().segment(nc_, nx_).setOnes();
  // terms then add their own placeholder blocks, so drop the assembled
  // off-diagonal values first
  if (P_ctrl_offdiag_)
    qp_.P.topLeftCorner(nc_, nc_).setZero();
  if (P_ctrl_.written)
    qp_.P.diagonal().head(nc_).setOnes();
}

void SparseFormulation::addControlCostPlaceholders(
    const Ref<const MatrixXd>& placeholders)
{
  qp_.P.topLeftCorner(nc_, nc_) += placeholders;
}

void SparseFormulation::predictStates(const Ref<const VectorXd>& z,
                                      Ref<VectorXd> x_traj) const noexcept
{
  x_traj = z.segment(nc_, nx_);
}

void SparseFormulation::endOwnCostMatrix()
{
  if (P_ctrl_offdiag_ && P_ctrl_.stale && P_ctrl_.written) {
    qp_.P.topLeftCorner(nc_, nc_).setZero();
    P_ctrl_offdiag_ = false;
  }
  finish(P_ctrl_, qp_.P.diagonal().head(nc_));
  finish(P_state_, qp_.P.diagonal().segment(nc_, nx_));
}

void SparseFormulation::endOwnCostVector()
{
  finish(q_ctrl_, qp_.q.head(nc_));
  finish(q_state_, qp_.q.segment(nc_, nx_));
}

void SparseFormulation::addStateQuadraticMatrix(const Ref<const VectorXd>& Q)
{
  assignOrAdd(P_state_, qp_.P.diagonal().segment(nc_, nx_), Q);
}

void SparseFormulation::addStateQuadraticVector(const Ref<const VectorXd>& Q,
                                                const Ref<const VectorXd>& r)
{
  // Products with a DiagonalMatrix copy it (a heap allocation) in Eigen 3.4
  assignOrAdd(q_state_, qp_.q.segment(nc_, nx_), -Q.cwiseProduct(r));
}

void SparseFormulation::addControlQuadraticMatrix(const Ref<const VectorXd>& R)
{
  // a diagonal write into a stale block must not leave old off-diagonals
  if (P_ctrl_.stale && P_ctrl_offdiag_) {
    qp_.P.topLeftCorner(nc_, nc_).setZero();
    P_ctrl_offdiag_ = false;
  }
  assignOrAdd(P_ctrl_, qp_.P.diagonal().head(nc_), R);
}

void SparseFormulation::addControlQuadraticVector(
    const Ref<const VectorXd>& R, const Ref<const VectorXd>& c_ref)
{
  assignOrAdd(q_ctrl_, qp_.q.head(nc_), -R.cwiseProduct(c_ref));
}

void SparseFormulation::addControlMatrix(const Ref<const MatrixXd>& P_c)
{
  assignOrAdd(P_ctrl_, qp_.P.topLeftCorner(nc_, nc_), P_c);
  P_ctrl_offdiag_ = true;
}

void SparseFormulation::addControlLinear(const Ref<const VectorXd>& q_c)
{
  assignOrAdd(q_ctrl_, qp_.q.head(nc_), q_c);
}

void SparseFormulation::setStateRows(const RowBlock& block,
                                     const StateRowMap& H,
                                     const int first_step,
                                     const int num_steps)
{
  // H on the columns of x_k; the control point columns and the rest of the
  // x_k columns are cleared (the other state columns are never written)
  const int p{H.rows()};
  for (int k{first_step}, row{block.offset}; k < first_step + num_steps;
       ++k, row += p) {
    const int col{nc_ + (k - 1) * n_};
    qp_.A.block(row, 0, p, nc_).setZero();
    qp_.A.block(row, col, p, n_).setZero();
    for (const StateRowMap::Entry& e : H.entries())
      qp_.A(row + e.row, col + e.state) = e.value;
  }
}

void SparseFormulation::setStateBounds(const RowBlock& block,
                                       const StateRowMap& H,
                                       const int /*first_step*/,
                                       const int num_steps,
                                       const Ref<const VectorXd>& lo,
                                       const Ref<const VectorXd>& hi)
{
  const int rows{H.rows() * num_steps};
  qp_.l.segment(block.offset, rows) = lo.replicate(num_steps, 1);
  qp_.u.segment(block.offset, rows) = hi.replicate(num_steps, 1);
}

void SparseFormulation::setDynamicsMatrixBlocks(const MPCData& d)
{
  // set model constraints that multiply controls
  const Parameterization& param{input_param_.parameterization()};
  for (int k{0}, row{0}; k < T_; ++k, row += n_) {
    for (int i{0}; i < param.band_width; ++i) {
      const int col{input_param_.coefficientIndex(0, param.band_start(k) + i)};
      qp_.A.block(row, col, n_, m_) = param.band_weights(i, k) * d.Bd;
    }
  }

  // set model constraints that multiply states
  for (int k{1}, row{n_}, col{nc_}; k < T_; ++k, row += n_, col += n_)
    qp_.A.block(row, col, n_, n_) = d.Ad;
}

void SparseFormulation::setDynamicsOffsetBounds(const MPCData& d)
{
  // k = 0 is written by updateX0() at every solve
  for (Index k{1}, offset{n_}; k < T_; ++k, offset += n_) {
    qp_.l.segment(offset, n_) = -d.wd;
    qp_.u.segment(offset, n_) = -d.wd;
  }
}

} // namespace affine_mpc::detail
