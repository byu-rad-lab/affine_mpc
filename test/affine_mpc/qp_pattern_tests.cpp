#include <Eigen/Core>
#include <Eigen/SparseCore>
#include <cmath>
#include <gtest/gtest.h>
#include <random>
#include <string>
#include <type_traits>
#include <vector>

#include "affine_mpc/condensed_mpc.hpp"
#include "affine_mpc/options.hpp"
#include "affine_mpc/osqp_solver.hpp"
#include "affine_mpc/parameterization.hpp"
#include "affine_mpc/quadratic_program.hpp"
#include "affine_mpc/sparse_mpc.hpp"
#include "qp_builder.hpp" // internal (src/): the declared entries
#include "qp_entries.hpp"
#include "utils.hpp"

namespace ampc = affine_mpc;
using namespace Eigen;

namespace {

using Mask = Matrix<bool, Dynamic, Dynamic>;

// Exposes the assembled QP and the entries declared at construction
class PatternInspector : public ampc::SparseMPC
{
public:
  using SparseMPC::SparseMPC;
  void assemble(const Ref<const VectorXd>& x0) { (void)this->qpUpdateX0(x0); }
  const ampc::detail::QP& qpData() const { return this->qp(); }
  const std::vector<ampc::detail::QPEntries*>& declaredP() const
  {
    return this->qpBuilder().declaredP();
  }
  const std::vector<ampc::detail::QPEntries*>& declaredA() const
  {
    return this->qpBuilder().declaredA();
  }

  // Nonzero entries of the QP outside every declared entry of P (A)
  std::vector<std::string> undeclaredNonzeros() const
  {
    std::vector<std::string> found;
    check("P", denseP(this->qp()), this->qpBuilder().declaredP(), found);
    check("A", denseA(this->qp()), this->qpBuilder().declaredA(), found);
    return found;
  }

private:
  static void check(const char* name,
                    const MatrixXd& M,
                    const std::vector<ampc::detail::QPEntries*>& sets,
                    std::vector<std::string>& found)
  {
    Mask declared{Mask::Constant(M.rows(), M.cols(), false)};
    for (const ampc::detail::QPEntries* entries : sets)
      for (Index i{0}; i < entries->size(); ++i)
        declared(entries->row(i), entries->col(i)) = true;
    for (Index j{0}; j < M.cols(); ++j)
      for (Index i{0}; i < M.rows(); ++i)
        if (M(i, j) != 0.0 && !declared(i, j))
          found.push_back(std::string{name} + "(" + std::to_string(i) + ", "
                          + std::to_string(j) + ")");
  }
};

// Random values with about a third of the entries zero, so entries switch
// between zero and nonzero from one draw to the next
class Draw
{
public:
  explicit Draw(unsigned seed) : gen_{seed} {}
  double value() { return std::uniform_real_distribution<>{-1.0, 1.0}(gen_); }
  bool coin(double p = 0.5) { return std::bernoulli_distribution{p}(gen_); }
  int integer(int lo, int hi)
  {
    return std::uniform_int_distribution<>{lo, hi}(gen_);
  }
  MatrixXd sparse(Index rows, Index cols)
  {
    MatrixXd M{rows, cols};
    for (Index j{0}; j < cols; ++j)
      for (Index i{0}; i < rows; ++i)
        M(i, j) = coin(1.0 / 3.0) ? 0.0 : value();
    return M;
  }
  // non-negative weights, some zero
  VectorXd weights(Index size) { return sparse(size, 1).cwiseAbs(); }

private:
  std::mt19937 gen_;
};

struct Case
{
  int n, m;
  ampc::Parameterization param;
  ampc::Options opts;
  int num_bounded;
};

Case drawCase(Draw& draw)
{
  using P = ampc::Parameterization;
  // custom bases: dense bands, with zeros (a B-spline's sampled basis) and
  // without (Laguerre), and a one-step horizon
  const std::vector<P> bases{P::moveBlocking(8, 8),
                             P::moveBlocking(12, 4),
                             P::linearInterp(10, 4),
                             P::bspline(10, 2, 5),
                             P::bspline(15, 3, 6),
                             P::moveBlocking(1, 1),
                             P::bspline(20, 2, 7, 1.3),
                             P::fromBasis(10, denseBasis(P::bspline(10, 2, 5))),
                             P::fromBasis(12, laguerreBasis(12, 4, 0.6)),
                             P::fromBasis(1, MatrixXd::Constant(1, 1, 0.8))};
  Case c{draw.integer(1, 4),
         draw.integer(1, 3),
         bases[static_cast<std::size_t>(
             draw.integer(0, static_cast<int>(bases.size()) - 1))],
         {},
         0};
  ampc::Options& o{c.opts};
  o.use_input_cost = draw.coin();
  o.use_input_rate_cost = draw.coin();
  o.saturate_input_trajectory = draw.coin();
  o.slew_initial_input = draw.coin();
  o.slew_control_points = draw.coin() && c.param.num_control_points > 1
                          && c.param.type == P::Type::BSpline;
  o.saturate_states = draw.coin();
  o.soft_state_bounds = o.saturate_states && draw.coin();
  if (o.saturate_states && draw.coin())
    for (int i{0}; i < c.n; ++i)
      if (draw.coin())
        o.bounded_states.push_back(i);
  o.num_general_constraints = draw.coin() ? draw.integer(1, 3) : 0;
  o.num_terminal_constraints = draw.coin() ? draw.integer(1, 3) : 0;
  c.num_bounded = o.bounded_states.empty()
                      ? c.n
                      : static_cast<int>(o.bounded_states.size());
  return c;
}

// Sets every value the options use, drawn anew
void setAll(ampc::MPCBase& mpc, const Case& c, Draw& draw)
{
  const int n{c.n}, m{c.m}, T{c.param.horizon_steps};
  const int nc{m * c.param.num_control_points};
  const ampc::Options& o{c.opts};
  mpc.setModelDiscrete(MatrixXd::Identity(n, n) + 0.1 * draw.sparse(n, n),
                       draw.sparse(n, m), draw.sparse(n, 1));
  mpc.setStateWeights(draw.weights(n), draw.weights(n));
  if (o.use_input_cost) {
    mpc.setInputWeights(draw.weights(m));
    mpc.setReferenceInputControlPoints(draw.sparse(nc, 1));
  }
  if (o.use_input_rate_cost)
    mpc.setInputRateWeights(draw.weights(m));
  mpc.setReferenceStateTrajectory(draw.sparse(n * T, 1));
  mpc.setInputLimits(VectorXd::Constant(m, -2.0), VectorXd::Constant(m, 2.0));
  const int k{c.num_bounded};
  if (o.saturate_states)
    mpc.setStateLimits(VectorXd::Constant(k, -5.0), VectorXd::Constant(k, 5.0));
  if (o.soft_state_bounds)
    mpc.setStateBoundPenalties(draw.weights(k), draw.weights(k));
  if (o.num_general_constraints > 0) {
    const int p{o.num_general_constraints};
    mpc.setGeneralConstraints(draw.sparse(p, n), draw.sparse(p, m),
                              VectorXd::Constant(p, -3.0),
                              VectorXd::Constant(p, 3.0));
  }
  if (o.num_terminal_constraints > 0) {
    const int p{o.num_terminal_constraints};
    mpc.setTerminalConstraints(draw.sparse(p, n), VectorXd::Constant(p, -3.0),
                               VectorXd::Constant(p, 3.0));
  }
  if (o.slew_control_points)
    mpc.setSlewRate(VectorXd::Constant(m, 0.7));
  if (o.slew_initial_input)
    mpc.setSlewRateInitial(VectorXd::Constant(m, 0.3));
  if (o.slew_initial_input || o.use_input_rate_cost)
    mpc.setPreviousInput(draw.sparse(m, 1));
}

std::string describe(const Case& c)
{
  const ampc::Options& o{c.opts};
  std::string s{"n" + std::to_string(c.n) + " m" + std::to_string(c.m) + " T"
                + std::to_string(c.param.horizon_steps) + " nc"
                + std::to_string(c.param.num_control_points) + " deg"
                + std::to_string(c.param.degree) + " opts "};
  for (const bool b :
       {o.use_input_cost, o.use_input_rate_cost, o.saturate_input_trajectory,
        o.slew_initial_input, o.slew_control_points, o.saturate_states,
        o.soft_state_bounds})
    s += b ? '1' : '0';
  s += " bounded " + std::to_string(o.bounded_states.size()) + " p"
       + std::to_string(o.num_general_constraints) + " pf"
       + std::to_string(o.num_terminal_constraints);
  return s;
}

} // namespace

// The sparse QP's nonzeros stay within the entries declared at construction,
// over many option sets and value changes (the pattern the sparse storage is
// built from)
TEST(QPPatternTester, givenSparseMPC_NonzerosAreWithinDeclaredEntries)
{
  for (unsigned seed{0}; seed < 200; ++seed) {
    Draw draw{seed};
    const Case c{drawCase(draw)};
    SCOPED_TRACE("seed " + std::to_string(seed) + ": " + describe(c));
    PatternInspector mpc{c.n, c.m, c.param, c.opts};
    setAll(mpc, c, draw);
    const VectorXd x0{draw.sparse(c.n, 1)};
    mpc.assemble(x0);
    EXPECT_EQ(mpc.undeclaredNonzeros(), std::vector<std::string>{})
        << "assembled";
    // every model entry tracked, since the drawn models change their zeros
    (void)mpc.initializeSolver(ampc::ModelSparsity::Full);
    EXPECT_EQ(mpc.undeclaredNonzeros(), std::vector<std::string>{})
        << "initialized";
    for (int change{0}; change < 4; ++change) {
      setAll(mpc, c, draw);
      mpc.assemble(draw.sparse(c.n, 1));
      EXPECT_EQ(mpc.undeclaredNonzeros(), std::vector<std::string>{})
          << "change " << change;
    }
    (void)mpc.solve(x0); // whatever the status, the QP was assembled
    EXPECT_EQ(mpc.undeclaredNonzeros(), std::vector<std::string>{}) << "solve";
  }
}

namespace {

using SparseMatrixXF = ampc::OSQPSolver::SparseMatrixXF;

// Exposes the CSC buffers the wrapper builds and the values it sends to OSQP
class SolverInspector : public ampc::OSQPSolver
{
public:
  using OSQPSolver::OSQPSolver;
  // the values of P (A) sent by the last initialize() or matrix update: the
  // stored values themselves when they go to OSQP without a copy
  VectorXd sentP(const SparseMatrixXF* P) const
  {
    if (P != nullptr && P_direct_)
      return Map<const VectorXd>{P->valuePtr(), P_nnz_};
    return P_x_;
  }
  VectorXd sentA(const SparseMatrixXF* A) const
  {
    if (A != nullptr && A_direct_)
      return Map<const VectorXd>{A->valuePtr(), A_nnz_};
    return A_x_;
  }
  const VectorXI& Pi() const { return P_i_; }
  const VectorXI& Pp() const { return P_p_; }
  const VectorXI& Ai() const { return A_i_; }
  const VectorXI& Ap() const { return A_p_; }
  bool direct() const { return P_direct_ && A_direct_; }
};

// The solver sends the CSC matrices as stored: their structure (P's upper
// triangle, all stored) as its pattern, and their values without a copy
void expectSentAsStored(const SolverInspector& csc, const ampc::detail::QP& qp)
{
  using MapI = Map<const ampc::OSQPSolver::VectorXI>;
  const Index nv{qp.P_csc.cols()};
  EXPECT_TRUE(
      expectEigenNear(csc.Pp(), MapI(qp.P_csc.outerIndexPtr(), nv + 1), 0.0))
      << "P cols";
  EXPECT_TRUE(expectEigenNear(
      csc.Pi(), MapI(qp.P_csc.innerIndexPtr(), qp.P_csc.nonZeros()), 0.0))
      << "P rows";
  EXPECT_TRUE(
      expectEigenNear(csc.Ap(), MapI(qp.A_csc.outerIndexPtr(), nv + 1), 0.0))
      << "A cols";
  EXPECT_TRUE(expectEigenNear(
      csc.Ai(), MapI(qp.A_csc.innerIndexPtr(), qp.A_csc.nonZeros()), 0.0))
      << "A rows";
  EXPECT_TRUE(csc.direct()) << "values copied";
}

} // namespace

// The sparse formulation's CSC pattern holds exactly the declared entries,
// each once: those of P on or above the diagonal (plus the diagonal, all
// declared), and all of A
TEST(QPPatternTester, givenSparseMPC_CscPatternIsTheDeclaredEntries)
{
  for (unsigned seed{0}; seed < 200; ++seed) {
    Draw draw{seed};
    const Case c{drawCase(draw)};
    SCOPED_TRACE("seed " + std::to_string(seed) + ": " + describe(c));
    PatternInspector mpc{c.n, c.m, c.param, c.opts};
    const ampc::detail::QP& qp{mpc.qpData()};
    ASSERT_EQ(qp.storage, ampc::detail::QPStorage::Sparse);
    EXPECT_EQ(qp.P.size(), 0);
    EXPECT_EQ(qp.A.size(), 0);
    ASSERT_TRUE(qp.P_csc.isCompressed());
    ASSERT_TRUE(qp.A_csc.isCompressed());

    auto expectPattern = [](const char* name, const SparseMatrixXF& M,
                            const std::vector<ampc::detail::QPEntries*>& sets,
                            bool upper) {
      SCOPED_TRACE(name);
      Mask declared{Mask::Constant(M.rows(), M.cols(), false)};
      for (const ampc::detail::QPEntries* entries : sets)
        for (Index i{0}; i < entries->size(); ++i)
          if (!upper || entries->row(i) <= entries->col(i))
            declared(entries->row(i), entries->col(i)) = true;
      if (upper) {
        EXPECT_TRUE(declared.diagonal().all()) << "undeclared diagonal";
      }
      Mask stored{Mask::Constant(M.rows(), M.cols(), false)};
      for (Index j{0}; j < M.outerSize(); ++j)
        for (SparseMatrixXF::InnerIterator it{M, j}; it; ++it) {
          EXPECT_FALSE(stored(it.row(), it.col())) << "stored twice";
          stored(it.row(), it.col()) = true;
        }
      EXPECT_TRUE(stored == declared);
      EXPECT_EQ(M.nonZeros(), declared.count());
    };
    expectPattern("P", qp.P_csc, mpc.declaredP(), true);
    expectPattern("A", qp.A_csc, mpc.declaredA(), false);
    // the dense blocks of A (Bd, Ad, Hu) are written column by column
    for (const ampc::detail::QPEntries* entries : mpc.declaredA())
      if (entries->hasBlocks()) {
        EXPECT_TRUE(entries->blockColumnsContiguous());
      }
  }
}

// What the OSQP wrapper sends for the sparse formulation's CSC matrices is
// their stored structure and values, over many option sets and value changes:
// every QP stays within the pattern, and the solutions match a dense solver
// initialized with the same values
TEST(QPPatternTester, givenSparseMPC_CscSolverDataIsTheStoredStructure)
{
  const OSQPSettings settings{ampc::OSQPSolver::getRecommendedSettings()};
  int initialized{0}, compared{0};
  for (unsigned seed{0}; seed < 200; ++seed) {
    Draw draw{seed};
    const Case c{drawCase(draw)};
    SCOPED_TRACE("seed " + std::to_string(seed) + ": " + describe(c));
    PatternInspector mpc{c.n, c.m, c.param, c.opts};
    setAll(mpc, c, draw);
    mpc.assemble(draw.sparse(c.n, 1));
    // every model entry tracked, since the drawn models change their zeros
    if (!mpc.initializeSolver(ampc::ModelSparsity::Full))
      continue;
    ++initialized;
    // the QP as sent by initializeSolver()
    const ampc::detail::QP& qp{mpc.qpData()};
    const int nv{static_cast<int>(qp.q.size())};
    const int nr{static_cast<int>(qp.l.size())};
    SolverInspector csc{nv, nr};
    ASSERT_TRUE(csc.initialize(qp.P_csc, qp.A_csc, qp.q, qp.l, qp.u, settings));
    expectSentAsStored(csc, qp);

    for (int change{0}; change < 4; ++change) {
      SCOPED_TRACE("change " + std::to_string(change));
      setAll(mpc, c, draw);
      mpc.assemble(draw.sparse(c.n, 1)); // also sent to the MPC's own solver
      // each kind of update in turn
      if (change % 2 == 0) {
        ASSERT_TRUE(csc.updateMatrices(qp.P_csc, qp.A_csc));
      } else {
        ASSERT_TRUE(csc.updateCostMatrix(qp.P_csc));
        ASSERT_TRUE(csc.updateConstraintMatrix(qp.A_csc));
      }
      expectSentAsStored(csc, qp);
      EXPECT_TRUE(csc.isWithinSparsityPattern(qp.P_csc, qp.A_csc));
      EXPECT_TRUE(mpc.isWithinSparsityPattern());

      ASSERT_TRUE(csc.updateCostVector(qp.q));
      ASSERT_TRUE(csc.updateBounds(qp.l, qp.u));
      csc.coldStart();
      SolverInspector dense{nv, nr};
      ASSERT_TRUE(
          dense.initialize(denseP(qp), denseA(qp), qp.q, qp.l, qp.u, settings));
      if (csc.solve() != ampc::SolveStatus::Success
          || dense.solve() != ampc::SolveStatus::Success)
        continue;
      ++compared;
      // the objective (zero weights can make the minimizer non-unique)
      const double obj{csc.getSolveInfo().objective};
      EXPECT_NEAR(obj, dense.getSolveInfo().objective,
                  1e-3 * (1.0 + std::abs(obj)));
    }
  }
  EXPECT_GT(initialized, 150);
  EXPECT_GT(compared, 300);
}

namespace {

// Exposes the internal QP and its assembly path, for either formulation
template <class Base> class QPInspector : public Base
{
public:
  using Base::Base;
  void assemble(const Ref<const VectorXd>& x0) { (void)this->qpUpdateX0(x0); }
  const ampc::detail::QP& qpData() const { return this->qp(); }
};

// The public QP equals the internal one (P's upper triangle, which is what
// the solver reads), P is exactly symmetric, and both matrices are compressed
// with no stored zero
void expectSameQP(const ampc::QuadraticProgram& qp, const ampc::detail::QP& d)
{
  const MatrixXd P{qp.P}, A{qp.A};
  EXPECT_TRUE(expectEigenNear(MatrixXd{P.triangularView<Upper>()},
                              MatrixXd{denseP(d).triangularView<Upper>()}, 0.0))
      << "P";
  EXPECT_TRUE(expectEigenNear(A, denseA(d), 0.0)) << "A";
  EXPECT_TRUE(expectEigenNear(P, MatrixXd{P.transpose()}, 0.0)) << "symmetric";
  EXPECT_TRUE(expectEigenNear(qp.q, d.q, 0.0)) << "q";
  EXPECT_TRUE(expectEigenNear(qp.l, d.l, 0.0)) << "l";
  EXPECT_TRUE(expectEigenNear(qp.u, d.u, 0.0)) << "u";
  for (const SparseMatrix<double>* M : {&qp.P, &qp.A}) {
    EXPECT_TRUE(M->isCompressed());
    for (Index k{0}; k < M->nonZeros(); ++k)
      EXPECT_NE(M->valuePtr()[k], 0.0);
  }
}

// The blocks start at 0, are contiguous, and cover `total`
void expectContiguous(const std::vector<ampc::QPBlock>& blocks, int total)
{
  int next{0};
  for (const ampc::QPBlock& b : blocks) {
    EXPECT_EQ(b.offset, next) << b.name;
    EXPECT_GT(b.size, 0) << b.name;
    next = b.offset + b.size;
  }
  EXPECT_EQ(next, total);
}

template <class MPC> void checkLayoutAndQP(const Case& c, Draw& draw)
{
  constexpr bool sparse{std::is_same_v<MPC, ampc::SparseMPC>};
  QPInspector<MPC> mpc{c.n, c.m, c.param, c.opts};
  setAll(mpc, c, draw);
  mpc.assemble(draw.sparse(c.n, 1));
  const ampc::QuadraticProgram qp{mpc.getQP()};
  expectSameQP(qp, mpc.qpData());
  EXPECT_EQ(qp.P.rows(), mpc.getNumQPVariables());
  EXPECT_EQ(qp.A.rows(), mpc.getNumQPConstraints());
  EXPECT_EQ(qp.A.cols(), mpc.getNumQPVariables());

  // the variables: control points, then the sparse states, then the slacks
  const ampc::Options& o{c.opts};
  const int T{c.param.horizon_steps}, k{c.num_bounded};
  const std::vector<ampc::QPBlock> vars{mpc.getQPVariableBlocks()};
  expectContiguous(vars, mpc.getNumQPVariables());
  std::vector<std::pair<std::string, int>> expected{
      {"control_points", c.m * c.param.num_control_points}};
  if (sparse)
    expected.push_back({"states", c.n * T});
  if (o.soft_state_bounds)
    expected.push_back({"state_slacks", k});
  ASSERT_EQ(vars.size(), expected.size());
  for (std::size_t i{0}; i < vars.size(); ++i) {
    EXPECT_EQ(vars[i].name, expected[i].first);
    EXPECT_EQ(vars[i].size, expected[i].second) << vars[i].name;
  }

  // the rows: only the enabled blocks, in the documented order (the input
  // bound rows depend on the bound rule, so only their presence is checked)
  const std::vector<ampc::QPBlock> rows{mpc.getQPConstraintBlocks()};
  expectContiguous(rows, mpc.getNumQPConstraints());
  expected.clear();
  if (sparse)
    expected.push_back({"dynamics", c.n * T});
  expected.push_back({"input_bounds", 0});
  if (o.slew_initial_input)
    expected.push_back({"initial_slew", c.m});
  if (o.slew_control_points)
    expected.push_back(
        {"control_point_slew", c.m * (c.param.num_control_points - 1)});
  if (o.saturate_states)
    expected.push_back(
        {"state_bounds", o.soft_state_bounds ? 2 * k * T + k : k * T});
  if (o.num_general_constraints > 0)
    expected.push_back({"general_constraints", o.num_general_constraints * T});
  if (o.num_terminal_constraints > 0)
    expected.push_back({"terminal_constraints", o.num_terminal_constraints});
  ASSERT_EQ(rows.size(), expected.size());
  for (std::size_t i{0}; i < rows.size(); ++i) {
    EXPECT_EQ(rows[i].name, expected[i].first);
    if (expected[i].second > 0) {
      EXPECT_EQ(rows[i].size, expected[i].second) << rows[i].name;
    }
  }
}

} // namespace

// getQP() returns the internal QP in the public form, and the variable and row
// blocks match the options, over many option sets and both formulations
TEST(QPAccessTester, givenAnyOptions_GetQPAndBlocksDescribeTheQP)
{
  for (unsigned seed{0}; seed < 100; ++seed) {
    Draw draw{seed};
    const Case c{drawCase(draw)};
    SCOPED_TRACE("seed " + std::to_string(seed) + ": " + describe(c));
    {
      SCOPED_TRACE("condensed");
      checkLayoutAndQP<ampc::CondensedMPC>(c, draw);
    }
    {
      SCOPED_TRACE("sparse");
      checkLayoutAndQP<ampc::SparseMPC>(c, draw);
    }
  }
}

namespace {

// Track every model entry where the formulation supports it (the drawn models
// change their zeros; CondensedMPC has no model check to trip)
template <class MPC> constexpr ampc::ModelSparsity allModelEntries()
{
  return std::is_same_v<MPC, ampc::SparseMPC>
             ? ampc::ModelSparsity::Full
             : ampc::ModelSparsity::InitialNonzeros;
}

template <class MPC> void checkAssemble(unsigned seed)
{
  // two MPCs set up with the same draws
  Draw draw_a{seed}, draw_b{seed};
  const Case c{drawCase(draw_a)};
  (void)drawCase(draw_b);
  QPInspector<MPC> a{c.n, c.m, c.param, c.opts}, b{c.n, c.m, c.param, c.opts};
  setAll(a, c, draw_a);
  setAll(b, c, draw_b);
  const VectorXd x0{draw_a.sparse(c.n, 1)};
  (void)draw_b.sparse(c.n, 1);

  // before initializeSolver(): the same QP as the internal assembly
  ampc::QuadraticProgram qp;
  a.assembleQP(x0, qp);
  b.assemble(x0);
  {
    SCOPED_TRACE("before initializeSolver()");
    expectSameQP(qp, b.qpData());
  }

  // after it, assembling for another x0 with new values pending leaves the
  // next solve as if assembleQP() had not been called
  ASSERT_EQ(a.initializeSolver(allModelEntries<MPC>()),
            b.initializeSolver(allModelEntries<MPC>()));
  if (!a.isSolverInitialized())
    return;
  setAll(a, c, draw_a);
  setAll(b, c, draw_b);
  a.assembleQP(draw_a.sparse(c.n, 1), qp);
  (void)draw_b.sparse(c.n, 1);
  const ampc::SolveStatus status_a{a.solve(x0)}, status_b{b.solve(x0)};
  EXPECT_EQ(status_a, status_b);
  EXPECT_EQ(a.getSolveInfo().iterations, b.getSolveInfo().iterations);
  VectorXd ctrl_a{c.m * c.param.num_control_points}, ctrl_b{ctrl_a.size()};
  a.getInputControlPoints(ctrl_a);
  b.getInputControlPoints(ctrl_b);
  EXPECT_TRUE(expectEigenNear(ctrl_a, ctrl_b, 0.0));
  // a failed update stops solve() before it writes x0, so each QP keeps the x0
  // parts last written (a's from assembleQP(), b's from initializeSolver())
  if (status_a != ampc::SolveStatus::UpdateFailed) {
    SCOPED_TRACE("after solve()");
    expectSameQP(a.getQP(), b.qpData());
  }
}

} // namespace

// assembleQP() gives the QP the solve path assembles and does not leave the
// solver out of date
TEST(QPAccessTester, givenAnyOptions_AssembleQPMatchesAndKeepsSolvesUnchanged)
{
  for (unsigned seed{0}; seed < 100; ++seed) {
    SCOPED_TRACE("seed " + std::to_string(seed));
    {
      SCOPED_TRACE("condensed");
      checkAssemble<ampc::CondensedMPC>(seed);
    }
    {
      SCOPED_TRACE("sparse");
      checkAssemble<ampc::SparseMPC>(seed);
    }
  }
}

namespace {

// Mass-spring-damper with every constraint kind (hard state bounds)
const ampc::Options constrained_msd_opts{.use_input_cost = true,
                                         .slew_initial_input = true,
                                         .slew_control_points = true,
                                         .saturate_states = true,
                                         .use_input_rate_cost = true,
                                         .num_general_constraints = 1,
                                         .num_terminal_constraints = 1};

void setConstrainedMsd(ampc::MPCBase& mpc)
{
  mpc.setModelContinuous2Discrete(Matrix2d{{0, 1}, {-0.6, -0.1}},
                                  Vector2d{0, 0.2}, Vector2d::Zero(), 0.1);
  mpc.setWeights(Vector2d{1, 0.1}, Matrix<double, 1, 1>{1e-3});
  mpc.setInputRateWeights(Matrix<double, 1, 1>{1e-2});
  mpc.setReferenceState(Vector2d{1, 0});
  mpc.setReferenceInput(Matrix<double, 1, 1>{3});
  mpc.setInputLimits(Matrix<double, 1, 1>{0}, Matrix<double, 1, 1>{4});
  mpc.setSlewRate(Matrix<double, 1, 1>{1});
  mpc.setSlewRateInitial(Matrix<double, 1, 1>{1});
  mpc.setPreviousInput(Matrix<double, 1, 1>{0});
  mpc.setStateLimits(Vector2d{-2, -2}, Vector2d{2, 2});
  mpc.setGeneralConstraints(RowVector2d{1, 1}, Matrix<double, 1, 1>{0.1},
                            Matrix<double, 1, 1>{-3}, Matrix<double, 1, 1>{3});
  mpc.setTerminalConstraints(RowVector2d{1, 0}, Matrix<double, 1, 1>{-2},
                             Matrix<double, 1, 1>{2});
}

template <class MPC> void checkSolutionAgainstQP()
{
  MPC mpc{2, 1, ampc::Parameterization::linearInterp(10, 4),
          constrained_msd_opts};
  setConstrainedMsd(mpc);
  ASSERT_TRUE(mpc.initializeSolver());
  ASSERT_EQ(mpc.solve(Vector2d{0.2, 0}), ampc::SolveStatus::Success);
  const ampc::QuadraticProgram qp{mpc.getQP()};

  // the solution in the order of the variable blocks
  VectorXd z{mpc.getNumQPVariables()};
  for (const ampc::QPBlock& b : mpc.getQPVariableBlocks()) {
    if (b.name == "control_points")
      mpc.getInputControlPoints(z.segment(b.offset, b.size));
    else if (b.name == "states")
      mpc.getPredictedStateTrajectory(z.segment(b.offset, b.size));
    else
      FAIL() << "unexpected block " << b.name;
  }
  const double objective{0.5 * z.dot(qp.P * z) + qp.q.dot(z)};
  EXPECT_NEAR(objective, mpc.getSolveInfo().objective,
              1e-9 * std::max(1.0, std::abs(objective)));
  const VectorXd Az{qp.A * z};
  EXPECT_TRUE(((Az - qp.l).array() >= -1e-5).all());
  EXPECT_TRUE(((qp.u - Az).array() >= -1e-5).all());
}

} // namespace

// The solution satisfies the returned QP's constraints and gives the solver's
// objective, so the QP and the variable order are the ones the solver used
TEST(QPAccessTester, givenSolve_SolutionFitsTheReturnedQP)
{
  {
    SCOPED_TRACE("condensed");
    checkSolutionAgainstQP<ampc::CondensedMPC>();
  }
  {
    SCOPED_TRACE("sparse");
    checkSolutionAgainstQP<ampc::SparseMPC>();
  }
}

TEST(QPAccessTester, givenMissingSetup_AssembleQPThrows)
{
  ampc::CondensedMPC mpc{2, 1, ampc::Parameterization::linearInterp(10, 4)};
  ampc::QuadraticProgram qp;
  expectLogicErrorWithMessage(
      [&] { mpc.assembleQP(Vector2d::Zero(), qp); },
      "[MPCBase::assembleQP] Model must be set before assembling the QP.");
  mpc.setModelDiscrete(Matrix2d::Identity(), Vector2d{0, 1}, Vector2d::Zero());
  expectLogicErrorWithMessage(
      [&] { mpc.assembleQP(Vector2d::Zero(), qp); },
      "[MPCBase::assembleQP] Input limits must be set before assembling the "
      "QP.");
}
