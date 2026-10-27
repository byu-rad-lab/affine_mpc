#include <Eigen/Core>
#include <Eigen/SparseCore>
#include <gtest/gtest.h>
#include <stdexcept>
#include <string>
#include <vector>

#include "affine_mpc/discretizer.hpp"
#include "affine_mpc/mpc.hpp"
#include "affine_mpc/options.hpp"
#include "affine_mpc/osqp_solver.hpp"
#include "affine_mpc/parameterization.hpp"
#include "qp_builder.hpp" // internal (src/): the declared entries
#include "qp_entries.hpp"
#include "utils.hpp"

namespace ampc = affine_mpc;
using namespace Eigen;

namespace {

using ampc::ModelSparsity;

constexpr int n{4}, m{2}, T{12}, nc{5};

// Exposes the QP and the entries declared for it
class Inspector : public ampc::MPC
{
public:
  using ampc::MPC::MPC;
  const ampc::detail::QP& qpData() const { return this->qp(); }
  const std::vector<ampc::detail::QPEntries*>& declaredA() const
  {
    return this->qpBuilder().declaredA();
  }
  int dynamicsRows() const { return this->qpBuilder().rows().dynamics.rows; }
};

// Two double integrators with a weak cross term: Ad and Bd have structural
// zeros. `tilt` scales entries that are zero at tilt = 0 (like the
// g sin(theta) terms of a quadrotor at hover).
void model(double tilt, MatrixXd& Ad, MatrixXd& Bd)
{
  // clang-format off
  Ad.resize(n, n);
  Ad << 1.0, 0.1, 0.0,        0.0,
        0.0, 1.0, 0.0,        tilt * 0.2,
        0.0, 0.0, 1.0,        0.1,
        0.0, 0.0, tilt * 0.3, 0.98;
  Bd.resize(n, m);
  Bd << 0.005,       0.0,
        0.1,         tilt * 0.05,
        0.0,         0.005,
        tilt * 0.02, 0.1;
  // clang-format on
}

const ampc::Options opts{.use_input_cost = true,
                         .slew_initial_input = true,
                         .state_bounds = ampc::StateBoundMode::Hard};

OSQPSettings tightSettings()
{
  OSQPSettings settings{ampc::OSQPSolver::getRecommendedSettings()};
  settings.eps_abs = settings.eps_rel = 1e-9;
  settings.max_iter = 20000;
  return settings;
}

// Everything but the model
void configure(ampc::MPC& mpc)
{
  mpc.setInputLimits(Vector2d::Constant(-1.0), Vector2d::Constant(1.0));
  mpc.setStateLimits(Vector4d::Constant(-3.0), Vector4d::Constant(3.0));
  mpc.setSlewRateInitial(Vector2d::Constant(0.5));
  mpc.setWeights(Vector4d{10.0, 1.0, 10.0, 1.0}, Vector2d::Constant(0.1));
  mpc.setReferenceState(Vector4d{1.0, 0.0, -0.5, 0.0});
}

VectorXd controlPoints(const ampc::MPC& mpc)
{
  VectorXd c{m * nc};
  mpc.getInputControlPoints(c);
  return c;
}

const ampc::Parameterization param{ampc::Parameterization::bspline(T, 2, nc)};

// Initialize with the model at `init_tilt`, then solve a sequence of models
// (tilts) from the same state, each from a cold start; compare against a
// sparse MPC that tracks every entry
template <ampc::Formulation F> void expectMatchesFull(double init_tilt)
{
  ampc::MPC mpc{n, m, param, opts, F};
  ampc::MPC full{n, m, param, opts, ampc::Formulation::Sparse};
  MatrixXd Ad, Bd;
  model(init_tilt, Ad, Bd);
  mpc.setModelDiscrete(Ad, Bd, Vector4d::Zero());
  full.setModelDiscrete(Ad, Bd, Vector4d::Zero());
  configure(mpc);
  configure(full);
  ASSERT_TRUE(mpc.initializeSolver(tightSettings()));
  ASSERT_TRUE(full.initializeSolver(tightSettings(), ModelSparsity::Full));
  const Vector4d x0{0.3, -0.1, 0.2, 0.05};
  for (const double tilt : {0.0, 0.5, -1.0, 1.0}) {
    SCOPED_TRACE("tilt " + std::to_string(tilt));
    model(tilt, Ad, Bd);
    for (ampc::MPC* each : std::vector<ampc::MPC*>{&mpc, &full}) {
      each->setModelDiscrete(Ad, Bd, Vector4d::Zero());
      each->resetWarmStart();
      ASSERT_EQ(each->solve(x0), ampc::SolveStatus::Success);
    }
    EXPECT_TRUE(mpc.isWithinSparsityPattern());
    EXPECT_TRUE(expectEigenNear(controlPoints(mpc), controlPoints(full), 1e-6));
  }
}

} // namespace

// Initialized with a representative model (every entry that may become
// nonzero is nonzero), the tracked initial nonzeros hold every later model, so
// both formulations solve the same problems as tracking every entry
TEST(ModelSparsityTester, givenRepresentativeInitModel_SparseMatchesFull)
{
  expectMatchesFull<ampc::Formulation::Sparse>(1.0);
}

TEST(ModelSparsityTester, givenRepresentativeInitModel_CondensedMatchesFull)
{
  expectMatchesFull<ampc::Formulation::Condensed>(1.0);
}

// With Full, entries that are zero at initialization are applied when they
// become nonzero
TEST(ModelSparsityTester, givenFullAndZerosAtInit_SparseAppliesLaterEntries)
{
  ampc::MPC mpc{n, m, param, opts, ampc::Formulation::Sparse},
      fresh{n, m, param, opts, ampc::Formulation::Sparse};
  MatrixXd Ad, Bd;
  model(0.0, Ad, Bd);
  mpc.setModelDiscrete(Ad, Bd, Vector4d::Zero());
  configure(mpc);
  ASSERT_TRUE(mpc.initializeSolver(tightSettings(), ModelSparsity::Full));
  EXPECT_EQ(mpc.getModelSparsity(), ModelSparsity::Full);
  const Vector4d x0{0.3, -0.1, 0.2, 0.05};
  ASSERT_EQ(mpc.solve(x0), ampc::SolveStatus::Success);

  model(1.0, Ad, Bd);
  mpc.setModelDiscrete(Ad, Bd, Vector4d::Zero());
  mpc.setPreviousInput(Vector2d::Zero()); // as in fresh
  mpc.resetWarmStart();
  fresh.setModelDiscrete(Ad, Bd, Vector4d::Zero());
  configure(fresh);
  ASSERT_TRUE(fresh.initializeSolver(tightSettings()));
  ASSERT_EQ(mpc.solve(x0), ampc::SolveStatus::Success);
  ASSERT_EQ(fresh.solve(x0), ampc::SolveStatus::Success);
  EXPECT_TRUE(mpc.isWithinSparsityPattern());
  EXPECT_TRUE(expectEigenNear(controlPoints(mpc), controlPoints(fresh), 1e-6));
}

// With InitialNonzeros (the default), entries that are zero at
// initialization are not tracked: both formulations report a later nonzero
TEST(ModelSparsityTester, givenInitialNonzerosAndZerosAtInit_LaterEntryReported)
{
  ampc::MPC sparse{n, m, param, opts, ampc::Formulation::Sparse};
  ampc::MPC condensed{n, m, param, opts};
  for (ampc::MPC* mpc : std::vector<ampc::MPC*>{&sparse, &condensed}) {
    SCOPED_TRACE(mpc->getClassName());
    MatrixXd Ad, Bd;
    model(0.0, Ad, Bd);
    mpc->setModelDiscrete(Ad, Bd, Vector4d::Zero());
    configure(*mpc);
    ASSERT_TRUE(mpc->initializeSolver());
    EXPECT_EQ(mpc->getModelSparsity(), ModelSparsity::InitialNonzeros);
    model(1.0, Ad, Bd);
#ifndef NDEBUG
    if (mpc == &sparse)
      continue; // its setter asserts (see the death test below)
#endif
    mpc->setModelDiscrete(Ad, Bd, Vector4d::Zero());
    (void)mpc->solve(Vector4d::Zero());
    EXPECT_FALSE(mpc->isWithinSparsityPattern());
  }
}

// The sparse formulation with InitialNonzeros stores (and so sends to OSQP)
// only the nonzero entries of Ad and Bd in the dynamics rows, at every step
TEST(ModelSparsityTester, givenInitialNonzeros_SparsePatternExcludesZeros)
{
  Inspector tracked{n, m, param, opts, ampc::Formulation::Sparse};
  Inspector full{n, m, param, opts, ampc::Formulation::Sparse};
  MatrixXd Ad, Bd;
  model(0.0, Ad, Bd);
  for (ampc::MPC* mpc : std::vector<ampc::MPC*>{&tracked, &full}) {
    mpc->setModelDiscrete(Ad, Bd, Vector4d::Zero());
    configure(*mpc);
  }
  ASSERT_TRUE(tracked.initializeSolver());
  ASSERT_TRUE(full.initializeSolver(ModelSparsity::Full));
  const Index zeros_A{(Ad.array() == 0.0).count()};
  const Index zeros_B{(Bd.array() == 0.0).count()};
  ASSERT_GT(zeros_A, 0);
  ASSERT_GT(zeros_B, 0);

  const auto& At{tracked.qpData().A_csc};
  const auto& Af{full.qpData().A_csc};
  // Bd at every step for each of the band's control points, Ad from step 1
  const Index band{param.band_width};
  EXPECT_EQ(Af.nonZeros() - At.nonZeros(),
            T * band * zeros_B + (T - 1) * zeros_A);
  EXPECT_EQ(full.qpData().P_csc.nonZeros(), tracked.qpData().P_csc.nonZeros());

  // every stored entry of the dynamics rows is a nonzero of the model (or the
  // -I of x_{k+1})
  const int nc_total{m * nc};
  for (Index j{0}; j < At.outerSize(); ++j)
    for (ampc::detail::QP::SparseMatrix::InnerIterator it{At, j}; it; ++it) {
      const Index row{it.row()}, col{it.col()};
      if (row >= tracked.dynamicsRows())
        continue;
      const Index i{row % n};
      if (col < nc_total) {
        // a Bd block: its column is the input of this control point column
        EXPECT_NE(Bd(i, col % m), 0.0) << row << ", " << col;
      } else {
        const Index state_col{col - nc_total}, k{row / n};
        if (state_col == row)
          continue; // -x_{k+1}
        ASSERT_EQ(state_col / n, k - 1) << row << ", " << col;
        EXPECT_NE(Ad(i, state_col % n), 0.0) << row << ", " << col;
      }
    }
}

// A model without zeros keeps the full blocks (their columns copied whole)
TEST(ModelSparsityTester, givenModelWithoutZeros_InitialNonzerosIsFull)
{
  Inspector tracked{n, m, param, opts, ampc::Formulation::Sparse};
  Inspector full{n, m, param, opts, ampc::Formulation::Sparse};
  for (ampc::MPC* mpc : std::vector<ampc::MPC*>{&tracked, &full}) {
    mpc->setModelDiscrete(MatrixXd::Constant(n, n, 0.1),
                          MatrixXd::Constant(n, m, 0.1), Vector4d::Zero());
    configure(*mpc);
  }
  ASSERT_TRUE(tracked.initializeSolver());
  ASSERT_TRUE(full.initializeSolver(ModelSparsity::Full));
  EXPECT_EQ(tracked.qpData().A_csc.nonZeros(), full.qpData().A_csc.nonZeros());
  for (const ampc::detail::QPEntries* entries : tracked.declaredA())
    if (entries->hasBlocks()) {
      EXPECT_TRUE(entries->blockColumnsContiguous());
    }
}

TEST(ModelSparsityTester, givenCondensedFull_InitializeSolverThrows)
{
  ampc::MPC mpc{n, m, param, opts};
  MatrixXd Ad, Bd;
  model(1.0, Ad, Bd);
  mpc.setModelDiscrete(Ad, Bd, Vector4d::Zero());
  configure(mpc);
  EXPECT_THROW((void)mpc.initializeSolver(ModelSparsity::Full),
               std::invalid_argument);
  EXPECT_FALSE(mpc.isSolverInitialized());
  EXPECT_TRUE(mpc.initializeSolver(ModelSparsity::InitialNonzeros));
}

// After initialization, a sparse-formulation model entry outside the tracked
// ones: debug builds assert in the model setters, and in Release
// isWithinSparsityPattern() reports it
TEST(ModelSparsityDeathTester,
     givenUntrackedModelEntry_DebugAssertsReleaseReports)
{
  MatrixXd Ad, Bd;
  model(1.0, Ad, Bd);
  MatrixXd Ad_out{Ad};
  Ad_out(0, 2) = 0.05; // zero at initialization
  MatrixXd Bd_out{Bd};
  Bd_out(2, 0) = 0.01; // zero at initialization

  ampc::MPC mpc{n, m, param, opts, ampc::Formulation::Sparse};
  mpc.setModelDiscrete(Ad, Bd, Vector4d::Zero());
  configure(mpc);
  ASSERT_TRUE(mpc.initializeSolver());
  ASSERT_EQ(mpc.solve(Vector4d::Zero()), ampc::SolveStatus::Success);
  EXPECT_TRUE(mpc.isWithinSparsityPattern());

  EXPECT_DEBUG_DEATH(mpc.setModelDiscrete(Ad_out, Bd, Vector4d::Zero()),
                     "solver does not track");
  EXPECT_DEBUG_DEATH(mpc.setModelDiscrete(Ad, Bd_out, Vector4d::Zero()),
                     "solver does not track");
  // the continuous model with the same coupling (Ac = Ad - I is enough)
  const MatrixXd Ac{Ad_out - MatrixXd::Identity(n, n)};
  EXPECT_DEBUG_DEATH(
      mpc.setModelContinuous2Discrete(Ac, Bd, Vector4d::Zero(), 0.1),
      "solver does not track");
#ifdef NDEBUG
  // the last setter ran: the model has untracked entries
  EXPECT_FALSE(mpc.isWithinSparsityPattern());
  mpc.setModelDiscrete(Ad, Bd, Vector4d::Zero());
  EXPECT_TRUE(mpc.isWithinSparsityPattern());
#endif
}

// A representative model as in the docs: ones at the union of the nonzeros
// over operating points of a continuous model. The discretization fills in
// the pattern of Ac (a chain of integrators gives an upper triangular Ad), so
// the union is taken over the discrete matrices.
TEST(ModelSparsityTester, givenUnionOverOperatingPoints_TracksEveryLaterModel)
{
  ampc::Discretizer discretizer{n, m};
  MatrixXd Ac{MatrixXd::Zero(n, n)}, Bc{MatrixXd::Zero(n, m)};
  MatrixXd Ad{n, n}, Bd{n, m};
  VectorXd wc{VectorXd::Zero(n)}, wd{n};
  MatrixXd Ad_union{MatrixXd::Zero(n, n)}, Bd_union{MatrixXd::Zero(n, m)};
  const auto setAc{[&](double gain) {
    // a chain of integrators driven at its end, with a state-dependent gain
    Ac(0, 1) = Ac(1, 2) = Ac(2, 3) = 1.0;
    Ac(3, 0) = gain;
    Bc(3, 0) = 1.0;
    Bc(1, 1) = gain;
  }};
  for (const double gain : {0.0, 0.5, -2.0}) {
    setAc(gain);
    discretizer.discretize(Ac, Bc, wc, 0.1, Ad, Bd, wd);
    Ad_union = (Ad.array() != 0.0).select(1.0, Ad_union);
    Bd_union = (Bd.array() != 0.0).select(1.0, Bd_union);
  }
  for (Index j{0}; j < n; ++j)
    for (Index i{0}; i <= j; ++i)
      EXPECT_EQ(Ad_union(i, j), 1.0) << i << ", " << j;
  EXPECT_GT((Ad_union.array() != 0.0).count(), (Ac.array() != 0.0).count());

  ampc::MPC mpc{n, m, param, opts, ampc::Formulation::Sparse};
  mpc.setModelDiscrete(Ad_union, Bd_union, wd);
  configure(mpc);
  ASSERT_TRUE(mpc.initializeSolver());
  for (const double gain : {0.0, 0.5, -2.0}) {
    setAc(gain);
    mpc.setModelContinuous2Discrete(Ac, Bc, wc, 0.1);
    EXPECT_EQ(mpc.solve(Vector4d::Zero()), ampc::SolveStatus::Success);
    EXPECT_TRUE(mpc.isWithinSparsityPattern());
  }
}
