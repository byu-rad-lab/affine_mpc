#include "affine_mpc/condensed_mpc.hpp"
#include "affine_mpc/options.hpp"
#include "affine_mpc/parameterization.hpp"
#include "affine_mpc/sparse_mpc.hpp"

#include <Eigen/Core>
#include <functional>
#include <gtest/gtest.h>
#include <limits>
#include <string>
#include <vector>

#include "utils.hpp"

using namespace Eigen;
namespace ampc = affine_mpc;

namespace {

// Exposes the assembled QP (P, A, q, l, u) and the pending change categories,
// and lets tests overwrite the QP matrices held by the MPC object.
template <class Formulation> class QPInspector : public Formulation
{
public:
  using Formulation::Formulation;
  void assemble(const Ref<const VectorXd>& x0)
  {
    ASSERT_TRUE(this->qpUpdateX0(x0));
  }
  ampc::ChangeSet pending() const { return this->pendingChanges(); }
  const MatrixXd& P() const { return this->P_; }
  const MatrixXd& A() const { return this->A_; }
  const VectorXd& q() const { return this->q_; }
  const VectorXd& l() const { return this->l_; }
  const VectorXd& u() const { return this->u_; }
  // Overwrites the local copies of the QP matrices; OSQP keeps its own copy,
  // so solves are unaffected unless the MPC rebuilds and sends them.
  void poisonMatrices()
  {
    const double nan{std::numeric_limits<double>::quiet_NaN()};
    this->P_.setConstant(nan);
    this->A_.setConstant(nan);
  }
};

template <class F>
void expectSameQP(const QPInspector<F>& actual, const QPInspector<F>& expected)
{
  // built from the same data with the same operations, so exactly equal
  EXPECT_TRUE(expectEigenNear(actual.P(), expected.P(), 0.0)) << "P";
  EXPECT_TRUE(expectEigenNear(actual.A(), expected.A(), 0.0)) << "A";
  EXPECT_TRUE(expectEigenNear(actual.q(), expected.q(), 0.0)) << "q";
  EXPECT_TRUE(expectEigenNear(actual.l(), expected.l(), 0.0)) << "l";
  EXPECT_TRUE(expectEigenNear(actual.u(), expected.u(), 0.0)) << "u";
}

// One value per setter group, so a single group can be changed
struct Scales
{
  double model{1.0}, offset{1.0}, weights{1.0}, refs{1.0};
  double input_limits{1.0}, state_limits{1.0}, slew{1.0}, slew0{1.0};
  double u_prev{1.0};
};

// SISO system with every entry of Ad and Bd nonzero (stays in the pattern)
void setModel(ampc::MPCBase& mpc, const Scales& s)
{
  Matrix2d Ad;
  Ad << 0.997, 0.0998, -0.0599, 0.987;
  const Vector2d Bd{0.001, 0.02}, wd{0.002, -0.01};
  mpc.setModelDiscrete(s.model * Ad, s.model * Bd, s.offset * wd);
}

void setOffset(ampc::MPCBase& mpc, const Scales& s)
{
  mpc.setModelOffsetDiscrete(s.offset * Vector2d{0.002, -0.01});
}

void setWeights(ampc::MPCBase& mpc, const Scales& s)
{
  mpc.setStateWeights(s.weights * Vector2d{1.0, 0.2},
                      s.weights * Vector2d{3.0, 0.4});
  if (mpc.getOptions().use_input_cost)
    mpc.setInputWeights(VectorXd::Constant(1, s.weights * 0.1));
}

void setRefs(ampc::MPCBase& mpc, const Scales& s)
{
  const int T{mpc.getHorizonSteps()}, nc{mpc.getNumControlPoints()};
  mpc.setReferenceStateTrajectory(s.refs
                                  * VectorXd::LinSpaced(2 * T, 0.5, 1.5));
  if (mpc.getOptions().use_input_cost)
    mpc.setReferenceInputControlPoints(s.refs
                                       * VectorXd::LinSpaced(nc, -0.3, 0.3));
}

void setInputLimits(ampc::MPCBase& mpc, const Scales& s)
{
  mpc.setInputLimits(VectorXd::Constant(1, -2.0 * s.input_limits),
                     VectorXd::Constant(1, 3.0 * s.input_limits));
}

void setStateLimits(ampc::MPCBase& mpc, const Scales& s)
{
  if (mpc.getOptions().saturate_states)
    mpc.setStateLimits(s.state_limits * Vector2d{-5.0, -6.0},
                       s.state_limits * Vector2d{7.0, 8.0});
}

void setSlew(ampc::MPCBase& mpc, const Scales& s)
{
  if (mpc.getOptions().slew_control_points)
    mpc.setSlewRate(VectorXd::Constant(1, 0.4 * s.slew));
}

void setSlew0(ampc::MPCBase& mpc, const Scales& s)
{
  if (mpc.getOptions().slew_initial_input)
    mpc.setSlewRateInitial(VectorXd::Constant(1, 0.5 * s.slew0));
}

void setPreviousInput(ampc::MPCBase& mpc, const Scales& s)
{
  if (mpc.getOptions().slew_initial_input)
    mpc.setPreviousInput(VectorXd::Constant(1, 0.1 * s.u_prev));
}

void configure(ampc::MPCBase& mpc, const Scales& s)
{
  setModel(mpc, s);
  setWeights(mpc, s);
  setRefs(mpc, s);
  setInputLimits(mpc, s);
  setStateLimits(mpc, s);
  setSlew(mpc, s);
  setSlew0(mpc, s);
  setPreviousInput(mpc, s);
}

struct CategoryCase
{
  std::string name;
  std::function<void(ampc::MPCBase&, const Scales&)> apply;
  Scales changed;
};

std::vector<CategoryCase> categoryCases()
{
  const double c{1.3};
  // clang-format off
  return {
      {"model", setModel, Scales{.model = c, .offset = c}},
      {"modelOffset", setOffset, Scales{.offset = c}},
      {"weights", setWeights, Scales{.weights = c}},
      {"references", setRefs, Scales{.refs = c}},
      {"inputLimits", setInputLimits, Scales{.input_limits = c}},
      {"stateLimits", setStateLimits, Scales{.state_limits = c}},
      {"slewRate", setSlew, Scales{.slew = c}},
      {"initialSlewRate", setSlew0, Scales{.slew0 = c}},
      {"previousInput", setPreviousInput, Scales{.u_prev = c}},
  };
  // clang-format on
}

// Changing one category of an initialized MPC must give exactly the QP of an
// MPC built from scratch with the new values, with or without prepare().
template <class F> void checkCategoriesAgainstFreshQP(const ampc::Options& opts)
{
  const int n{2}, m{1};
  const auto param{ampc::Parameterization::bspline(10, 2, 5)};
  const Vector2d x0{0.3, -0.1};

  for (const CategoryCase& c : categoryCases()) {
    for (const bool use_prepare : {false, true}) {
      SCOPED_TRACE(c.name + (use_prepare ? " with prepare()" : ""));
      QPInspector<F> mpc{n, m, param, opts};
      configure(mpc, Scales{});
      ASSERT_TRUE(mpc.initializeSolver());
      mpc.assemble(x0);

      c.apply(mpc, c.changed);
      if (use_prepare) {
        ASSERT_EQ(mpc.prepare(), ampc::SolveStatus::Success);
        EXPECT_TRUE(mpc.pending().empty());
      }
      mpc.assemble(x0);

      QPInspector<F> fresh{n, m, param, opts};
      configure(fresh, c.changed);
      ASSERT_TRUE(fresh.initializeSolver());
      fresh.assemble(x0);

      expectSameQP(mpc, fresh);
    }
  }
}

const ampc::Options all_options{.use_input_cost = true,
                                .slew_initial_input = true,
                                .slew_control_points = true,
                                .saturate_states = true};

const ampc::Options sampled_bounds{.use_input_cost = true,
                                   .saturate_input_trajectory = true};

} // namespace

TEST(ChangeCategoryTester,
     givenEachCategoryChanged_CondensedQPMatchesFreshlyBuiltQP)
{
  for (const ampc::Options& opts :
       {ampc::Options{}, all_options, sampled_bounds}) {
    checkCategoriesAgainstFreshQP<ampc::CondensedMPC>(opts);
  }
}

TEST(ChangeCategoryTester,
     givenEachCategoryChanged_SparseQPMatchesFreshlyBuiltQP)
{
  for (const ampc::Options& opts :
       {ampc::Options{}, all_options, sampled_bounds}) {
    checkCategoriesAgainstFreshQP<ampc::SparseMPC>(opts);
  }
}

TEST(ChangeCategoryTester, givenSetters_DoNotChangeTheQP)
{
  auto check = [](auto& mpc) {
    configure(mpc, Scales{});
    ASSERT_TRUE(mpc.initializeSolver());
    mpc.assemble(Vector2d{0.3, -0.1});
    const MatrixXd P{mpc.P()}, A{mpc.A()};
    const VectorXd q{mpc.q()}, l{mpc.l()}, u{mpc.u()};

    const Scales changed{.model = 1.3,
                         .offset = 1.3,
                         .weights = 1.3,
                         .refs = 1.3,
                         .input_limits = 1.3,
                         .state_limits = 1.3,
                         .slew = 1.3,
                         .slew0 = 1.3,
                         .u_prev = 1.3};
    configure(mpc, changed);
    EXPECT_EQ(mpc.pending(), ampc::ChangeSet::all());
    EXPECT_TRUE(expectEigenNear(mpc.P(), P, 0.0));
    EXPECT_TRUE(expectEigenNear(mpc.A(), A, 0.0));
    EXPECT_TRUE(expectEigenNear(mpc.q(), q, 0.0));
    EXPECT_TRUE(expectEigenNear(mpc.l(), l, 0.0));
    EXPECT_TRUE(expectEigenNear(mpc.u(), u, 0.0));
  };
  const auto param{ampc::Parameterization::bspline(10, 2, 5)};
  {
    SCOPED_TRACE("CondensedMPC");
    QPInspector<ampc::CondensedMPC> condensed{2, 1, param, all_options};
    check(condensed);
  }
  {
    SCOPED_TRACE("SparseMPC");
    QPInspector<ampc::SparseMPC> sparse{2, 1, param, all_options};
    check(sparse);
  }
}

// ---- prepare() --------------------------------------------------------------

TEST(PrepareTester, givenUninitializedSolver_ReturnsNotInitialized)
{
  ampc::CondensedMPC condensed{2, 1, 10};
  ampc::SparseMPC sparse{2, 1, 10};
  EXPECT_EQ(condensed.prepare(), ampc::SolveStatus::NotInitialized);
  EXPECT_EQ(sparse.prepare(), ampc::SolveStatus::NotInitialized);
  EXPECT_EQ(condensed.getSolveInfo().prepare_time, 0.0);
}

// prepare() followed by solve(x0) must give exactly the solution of solve(x0)
// alone, over a sequence of solves that change every category.
template <class F> void checkPrepareMatchesSolveAlone(const ampc::Options& opts)
{
  const int n{2}, m{1}, T{10};
  const auto param{ampc::Parameterization::bspline(T, 2, 5)};
  F prepared{n, m, param, opts};
  F direct{n, m, param, opts};
  configure(prepared, Scales{});
  configure(direct, Scales{});
  ASSERT_TRUE(prepared.initializeSolver());
  ASSERT_TRUE(direct.initializeSolver());

  Vector2d x0{0.3, -0.1};
  for (int step{0}; step < 6; ++step) {
    SCOPED_TRACE(::testing::Message() << "step " << step);
    // change a different set of categories each step (step 0 only applies the
    // weights pending after initialization)
    const double s{1.0 + 0.1 * step};
    for (ampc::MPCBase* mpc : {static_cast<ampc::MPCBase*>(&prepared),
                               static_cast<ampc::MPCBase*>(&direct)}) {
      if (step == 1 || step == 4)
        setModel(*mpc, Scales{.model = 1.0 + 0.01 * step, .offset = s});
      if (step == 2)
        setWeights(*mpc, Scales{.weights = s});
      if (step == 3) {
        setOffset(*mpc, Scales{.offset = s});
        setRefs(*mpc, Scales{.refs = s});
        setInputLimits(*mpc, Scales{.input_limits = s});
      }
      if (step == 4)
        setWeights(*mpc, Scales{.weights = 1.0 / s});
    }

    ASSERT_EQ(prepared.prepare(), ampc::SolveStatus::Success);
    const ampc::SolveStatus status_prepared{prepared.solve(x0)};
    const ampc::SolveStatus status_direct{direct.solve(x0)};
    ASSERT_EQ(status_prepared, ampc::SolveStatus::Success);
    ASSERT_EQ(status_direct, status_prepared);

    VectorXd z_prepared{m * param.num_control_points};
    VectorXd z_direct{m * param.num_control_points};
    prepared.getInputControlPoints(z_prepared);
    direct.getInputControlPoints(z_direct);
    EXPECT_TRUE(expectEigenNear(z_prepared, z_direct, 0.0));
    EXPECT_EQ(prepared.getSolveInfo().iterations,
              direct.getSolveInfo().iterations);
    // a model change makes prepare() rebuild matrices, so it takes time
    if (step == 1 || step == 4) {
      EXPECT_GT(prepared.getSolveInfo().prepare_time, 0.0);
    }
    EXPECT_EQ(direct.getSolveInfo().prepare_time, 0.0);

    VectorXd u0{m};
    prepared.getNextInput(u0);
    prepared.propagateModel(x0, u0, x0);
  }
}

TEST(PrepareTester, givenPrepareBeforeSolve_CondensedMatchesSolveAlone)
{
  for (const ampc::Options& opts : {ampc::Options{}, all_options})
    checkPrepareMatchesSolveAlone<ampc::CondensedMPC>(opts);
}

TEST(PrepareTester, givenPrepareBeforeSolve_SparseMatchesSolveAlone)
{
  for (const ampc::Options& opts : {ampc::Options{}, all_options})
    checkPrepareMatchesSolveAlone<ampc::SparseMPC>(opts);
}

// After prepare(), solve(x0) must not rebuild or send QP matrices: overwriting
// the MPC's local copies of P and A with NaN must leave them untouched and the
// solution unchanged (OSQP keeps its own copy of the prepared matrices).
template <class F> void checkSolveAfterPrepareDoesNoMatrixWork()
{
  const int n{2}, m{1}, T{10};
  const auto param{ampc::Parameterization::bspline(T, 2, 5)};
  QPInspector<F> prepared{n, m, param, all_options};
  QPInspector<F> reference{n, m, param, all_options};
  configure(prepared, Scales{});
  configure(reference, Scales{});
  ASSERT_TRUE(prepared.initializeSolver());
  ASSERT_TRUE(reference.initializeSolver());
  const Vector2d x0{0.3, -0.1};
  ASSERT_EQ(prepared.solve(x0), ampc::SolveStatus::Success);
  ASSERT_EQ(reference.solve(x0), ampc::SolveStatus::Success);

  // every category, including the matrix ones
  const Scales changed{.model = 1.1, .offset = 1.2, .weights = 1.3};
  configure(prepared, changed);
  configure(reference, changed);

  ASSERT_EQ(prepared.prepare(), ampc::SolveStatus::Success);
  EXPECT_TRUE(prepared.pending().empty());
  prepared.poisonMatrices();
  ASSERT_EQ(prepared.solve(x0), ampc::SolveStatus::Success);
  ASSERT_EQ(reference.solve(x0), ampc::SolveStatus::Success);

  EXPECT_TRUE(prepared.P().array().isNaN().all()) << "P was rebuilt";
  EXPECT_TRUE(prepared.A().array().isNaN().all()) << "A was rebuilt";
  VectorXd z_prepared{m * param.num_control_points};
  VectorXd z_reference{m * param.num_control_points};
  prepared.getInputControlPoints(z_prepared);
  reference.getInputControlPoints(z_reference);
  EXPECT_TRUE(expectEigenNear(z_prepared, z_reference, 0.0));

  // control: without prepare(), solve() rebuilds P after a weight change (the
  // solve itself may fail on the other NaN entries)
  setWeights(prepared, Scales{});
  (void)prepared.solve(x0);
  EXPECT_FALSE(prepared.P().diagonal().array().isNaN().any());
}

TEST(PrepareTester, givenPrepare_CondensedSolveDoesNoMatrixWork)
{
  checkSolveAfterPrepareDoesNoMatrixWork<ampc::CondensedMPC>();
}

TEST(PrepareTester, givenPrepare_SparseSolveDoesNoMatrixWork)
{
  checkSolveAfterPrepareDoesNoMatrixWork<ampc::SparseMPC>();
}

TEST(PrepareTester, givenNonFiniteUpdate_PrepareReportsUpdateFailed)
{
  auto check = [](ampc::MPCBase& mpc) {
    configure(mpc, Scales{});
    ASSERT_TRUE(mpc.initializeSolver());
    const Vector2d x0{0.3, -0.1};
    ASSERT_EQ(mpc.solve(x0), ampc::SolveStatus::Success);

    const double nan{std::numeric_limits<double>::quiet_NaN()};
    mpc.setStateWeights(Vector2d{nan, 1.0});
    EXPECT_EQ(mpc.prepare(), ampc::SolveStatus::UpdateFailed);
    // the failed update stays pending, so solve() retries and reports it
    EXPECT_EQ(mpc.solve(x0), ampc::SolveStatus::UpdateFailed);

    setWeights(mpc, Scales{});
    EXPECT_EQ(mpc.prepare(), ampc::SolveStatus::Success);
    EXPECT_EQ(mpc.solve(x0), ampc::SolveStatus::Success);
  };
  const auto param{ampc::Parameterization::bspline(10, 2, 5)};
  {
    SCOPED_TRACE("CondensedMPC");
    ampc::CondensedMPC condensed{2, 1, param, all_options};
    check(condensed);
  }
  {
    SCOPED_TRACE("SparseMPC");
    ampc::SparseMPC sparse{2, 1, param, all_options};
    check(sparse);
  }
}
