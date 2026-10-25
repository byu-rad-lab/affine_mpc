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

// Exposes the assembled QP (P, A, q, l, u) and the pending change categories.
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
// MPC built from scratch with the new values.
template <class F> void checkCategoriesAgainstFreshQP(const ampc::Options& opts)
{
  const int n{2}, m{1};
  const auto param{ampc::Parameterization::bspline(10, 2, 5)};
  const Vector2d x0{0.3, -0.1};

  for (const CategoryCase& c : categoryCases()) {
    {
      SCOPED_TRACE(c.name);
      QPInspector<F> mpc{n, m, param, opts};
      configure(mpc, Scales{});
      ASSERT_TRUE(mpc.initializeSolver());
      mpc.assemble(x0);

      c.apply(mpc, c.changed);
      mpc.assemble(x0);
      EXPECT_TRUE(mpc.pending().empty());

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
