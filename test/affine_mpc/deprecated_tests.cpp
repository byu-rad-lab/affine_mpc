// Tests of the deprecated names CondensedMPC, SparseMPC, and MPCBase (removed
// in 0.5.0), which use them on purpose. That the names warn is checked by the
// deprecated_*_warns tests (see CMakeLists.txt and deprecated_use.cpp).
#if defined(__GNUC__) || defined(__clang__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#elif defined(_MSC_VER)
#pragma warning(push)
#pragma warning(disable : 4996)
#endif

#include <Eigen/Core>
#include <gtest/gtest.h>
#include <sstream>
#include <string>
#include <type_traits>

#include "affine_mpc/affine_mpc.hpp"
#include "utils.hpp"

namespace ampc = affine_mpc;
using namespace Eigen;

namespace {

static_assert(std::is_same_v<ampc::MPCBase, ampc::MPC>);
static_assert(std::is_base_of_v<ampc::MPC, ampc::CondensedMPC>);
static_assert(std::is_base_of_v<ampc::MPC, ampc::SparseMPC>);
static_assert(std::is_final_v<ampc::CondensedMPC>);
static_assert(std::is_final_v<ampc::SparseMPC>);
static_assert(std::has_virtual_destructor_v<ampc::MPC>);

const auto param{ampc::Parameterization::linearInterp(10, 4)};
const ampc::Options opts{.use_input_cost = true};

// Mass-spring-damper asked to move to position 1
VectorXd solveMsd(ampc::MPC& mpc)
{
  Matrix2d Ac;
  Ac << 0, 1, -0.6, -0.1;
  mpc.setModelContinuous2Discrete(Ac, Vector2d{0, 0.2}, Vector2d::Zero(), 0.1);
  mpc.setWeights(Vector2d{1.0, 0.1}, VectorXd::Constant(1, 1e-3));
  mpc.setReferenceState(Vector2d{1.0, 0.0});
  mpc.setInputLimits(VectorXd::Constant(1, -3.0), VectorXd::Constant(1, 3.0));
  EXPECT_TRUE(mpc.initializeSolver());
  EXPECT_EQ(mpc.solve(Vector2d::Zero()), ampc::SolveStatus::Success);
  VectorXd c{mpc.getNumControlPoints()};
  mpc.getInputControlPoints(c);
  return c;
}

} // namespace

TEST(DeprecatedNamesTester, givenCondensedMPC_IsMPCWithCondensedFormulation)
{
  ampc::CondensedMPC old_param{2, 1, param, opts};
  ampc::CondensedMPC old_steps{2, 1, 10};
  EXPECT_EQ(old_param.getFormulation(), ampc::Formulation::Condensed);
  EXPECT_EQ(old_steps.getFormulation(), ampc::Formulation::Condensed);
  EXPECT_EQ(old_steps.getNumControlPoints(), 10);
  EXPECT_STREQ(old_param.getClassName(), "CondensedMPC");

  ampc::MPC mpc{2, 1, param, opts};
  EXPECT_TRUE(expectEigenNear(solveMsd(old_param), solveMsd(mpc), 0.0));

  std::ostringstream ss;
  ss << old_param;
  EXPECT_EQ(ss.str().rfind("CondensedMPC:\n  formulation = Condensed\n", 0),
            0U);
}

TEST(DeprecatedNamesTester, givenSparseMPC_IsMPCWithSparseFormulation)
{
  ampc::SparseMPC old_param{2, 1, param, opts};
  ampc::SparseMPC old_steps{2, 1, 10};
  EXPECT_EQ(old_param.getFormulation(), ampc::Formulation::Sparse);
  EXPECT_EQ(old_steps.getFormulation(), ampc::Formulation::Sparse);
  EXPECT_EQ(old_steps.getNumControlPoints(), 10);
  EXPECT_STREQ(old_param.getClassName(), "SparseMPC");

  ampc::MPC mpc{2, 1, param, opts, ampc::Formulation::Sparse};
  EXPECT_TRUE(expectEigenNear(solveMsd(old_param), solveMsd(mpc), 0.0));
}

TEST(DeprecatedNamesTester, givenMPCBaseReference_BindsToEitherShim)
{
  ampc::CondensedMPC condensed{2, 1, param};
  ampc::SparseMPC sparse{2, 1, param};
  for (ampc::MPCBase* mpc : {static_cast<ampc::MPCBase*>(&condensed),
                             static_cast<ampc::MPCBase*>(&sparse)})
    EXPECT_EQ(mpc->getHorizonSteps(), 10);
}

#if defined(__GNUC__) || defined(__clang__)
#pragma GCC diagnostic pop
#elif defined(_MSC_VER)
#pragma warning(pop)
#endif
