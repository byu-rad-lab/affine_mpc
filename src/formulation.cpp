#include "formulation.hpp"

#include <Eigen/Core>

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

} // namespace

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

void CondensedFormulation::updateX0(const MPCData& d,
                                    const Ref<const VectorXd>& x0)
{
  updateV(d, x0);
}

void CondensedFormulation::predictStates(const Ref<const VectorXd>& z,
                                         Ref<VectorXd> x_traj) const noexcept
{
  x_traj.noalias() = S_ * z;
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
    qp_.P.noalias() = S_.transpose() * weighted_S_;
  else
    qp_.P.noalias() += S_.transpose() * weighted_S_;
  P_.stale = false;
  P_.written = true;
}

void CondensedFormulation::addStateQuadraticVector(const Ref<const VectorXd>& Q,
                                                   const Ref<const VectorXd>& r)
{
  // q = S^T diag(Q) (v - r)
  weighted_state_err_.noalias() = Q.cwiseProduct(v_ - r);
  if (q_.stale)
    qp_.q.noalias() = S_.transpose() * weighted_state_err_;
  else
    qp_.q.noalias() += S_.transpose() * weighted_state_err_;
  q_.stale = false;
  q_.written = true;
}

void CondensedFormulation::addControlQuadraticMatrix(
    const Ref<const VectorXd>& R)
{
  startPartialWrite(P_, qp_.P);
  qp_.P.diagonal().head(nc_) += R;
}

void CondensedFormulation::addControlQuadraticVector(
    const Ref<const VectorXd>& R, const Ref<const VectorXd>& c_ref)
{
  startPartialWrite(q_, qp_.q);
  qp_.q.head(nc_).noalias() -= R.cwiseProduct(c_ref);
}

void CondensedFormulation::setStateBoxRows(const RowBlock& block)
{
  qp_.A.middleRows(block.offset, nx_) = S_;
}

void CondensedFormulation::setStateBoxBounds(const RowBlock& block,
                                             const Ref<const VectorXd>& lo,
                                             const Ref<const VectorXd>& hi)
{
  // lo <= S c + v <= hi becomes lo - v <= S c <= hi - v
  qp_.l.segment(block.offset, nx_) = lo.replicate(T_, 1) - v_;
  qp_.u.segment(block.offset, nx_) = hi.replicate(T_, 1) - v_;
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

void CondensedFormulation::updateV(const MPCData& d,
                                   const Ref<const VectorXd>& x0)
{
  // add dynamics of initial time step (k=0). Adding wd separately avoids a
  // temporary for the product inside a sum.
  v_.head(n_).noalias() = d.Ad * x0;
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

void SparseFormulation::updateX0(const MPCData& d,
                                 const Ref<const VectorXd>& x0)
{
  // the k = 0 dynamics rows: -x_1 + Bd u_0 = -Ad x0 - wd. Separate statements
  // avoid a temporary for the product inside a sum.
  qp_.l.head(n_) = -d.wd;
  qp_.l.head(n_).noalias() -= d.Ad * x0;
  qp_.u.head(n_) = qp_.l.head(n_);
}

DataChangeSet SparseFormulation::writeCostPattern()
{
  // Store every weight position on P's diagonal so a weight that is zero now
  // can become nonzero later. This costs nothing in solves since OSQP's KKT
  // system always includes P's diagonal. The weights are re-applied after
  // initialization.
  qp_.P.diagonal().segment(nc_, nx_).setOnes();
  if (P_ctrl_.written)
    qp_.P.diagonal().head(nc_).setOnes();
  return DataChange::Weights;
}

void SparseFormulation::predictStates(const Ref<const VectorXd>& z,
                                      Ref<VectorXd> x_traj) const noexcept
{
  x_traj = z.tail(nx_);
}

void SparseFormulation::endCostMatrix()
{
  finish(P_ctrl_, qp_.P.diagonal().head(nc_));
  finish(P_state_, qp_.P.diagonal().segment(nc_, nx_));
}

void SparseFormulation::endCostVector()
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
  assignOrAdd(P_ctrl_, qp_.P.diagonal().head(nc_), R);
}

void SparseFormulation::addControlQuadraticVector(
    const Ref<const VectorXd>& R, const Ref<const VectorXd>& c_ref)
{
  assignOrAdd(q_ctrl_, qp_.q.head(nc_), -R.cwiseProduct(c_ref));
}

void SparseFormulation::setStateBoxRows(const RowBlock& block)
{
  qp_.A.block(block.offset, nc_, nx_, nx_).diagonal().setOnes();
}

void SparseFormulation::setStateBoxBounds(const RowBlock& block,
                                          const Ref<const VectorXd>& lo,
                                          const Ref<const VectorXd>& hi)
{
  qp_.l.segment(block.offset, nx_) = lo.replicate(T_, 1);
  qp_.u.segment(block.offset, nx_) = hi.replicate(T_, 1);
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
