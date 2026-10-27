#include "affine_mpc/mpc.hpp"
#include "affine_mpc/options.hpp"
#include "affine_mpc/parameterization.hpp"

#include <Eigen/Core>
#include <cstddef>
#include <gtest/gtest.h>
#include <string>

using namespace Eigen;
namespace ampc = affine_mpc;

// Counts heap allocations by interposing the C allocation functions, which
// operator new, Eigen, and OSQP all use. The definitions in this executable
// replace glibc's for every shared library, and forward to glibc's internal
// implementations. Sanitizers interpose these functions themselves, so the
// count is disabled under them.
#if defined(__has_feature)
#if __has_feature(address_sanitizer) || __has_feature(thread_sanitizer)        \
    || __has_feature(memory_sanitizer)
#define AMPC_SANITIZER_MALLOC
#endif
#endif
#if defined(__SANITIZE_ADDRESS__) || defined(__SANITIZE_THREAD__)
#define AMPC_SANITIZER_MALLOC
#endif

#if defined(__GLIBC__) && !defined(AMPC_SANITIZER_MALLOC)
#define AMPC_COUNT_ALLOCATIONS
#endif

namespace {
bool counting{false};
std::size_t num_allocations{0};
} // namespace

#ifdef AMPC_COUNT_ALLOCATIONS
extern "C" {
void* __libc_malloc(std::size_t size);
void* __libc_calloc(std::size_t num, std::size_t size);
void* __libc_realloc(void* ptr, std::size_t size);
void* __libc_memalign(std::size_t alignment, std::size_t size);

void* malloc(std::size_t size)
{
  if (counting)
    ++num_allocations;
  return __libc_malloc(size);
}

void* calloc(std::size_t num, std::size_t size)
{
  if (counting)
    ++num_allocations;
  return __libc_calloc(num, size);
}

void* realloc(void* ptr, std::size_t size)
{
  if (counting)
    ++num_allocations;
  return __libc_realloc(ptr, size);
}

void* aligned_alloc(std::size_t alignment, std::size_t size)
{
  if (counting)
    ++num_allocations;
  return __libc_memalign(alignment, size);
}

int posix_memalign(void** ptr, std::size_t alignment, std::size_t size)
{
  if (counting)
    ++num_allocations;
  *ptr = __libc_memalign(alignment, size);
  return *ptr ? 0 : 12; // ENOMEM
}
}
#endif

namespace {

// Number of heap allocations made by fn
template <typename Fn> std::size_t countAllocations(Fn&& fn)
{
  num_allocations = 0;
  counting = true;
  fn();
  counting = false;
  return num_allocations;
}

template <ampc::Formulation F>
void checkNoAllocationsAfterInitialize(const ampc::Parameterization& param,
                                       const ampc::Options& opts,
                                       bool use_prepare)
{
#ifndef AMPC_COUNT_ALLOCATIONS
  GTEST_SKIP() << "allocation counting needs glibc and no sanitizer";
#endif
  const int n{2}, m{1};
  ampc::MPC mpc{n, m, param, opts, F};
  Matrix2d Ac;
  Ac << 0, 1, -0.6, -0.1;
  const Vector2d Bc{0, 0.2};
  mpc.setModelContinuous2Discrete(Ac, Bc, Vector2d::Zero(), 0.1);
  mpc.setInputLimits(VectorXd::Constant(m, -3.0), VectorXd::Constant(m, 3.0));
  mpc.setStateWeights(Vector2d{1.0, 0.1});
  mpc.setReferenceState(Vector2d{1.0, 0.0});
  if (opts.use_input_cost) {
    mpc.setInputWeights(VectorXd::Constant(m, 1e-3));
    mpc.setReferenceInput(VectorXd::Zero(m));
  }
  if (opts.slew_initial_input)
    mpc.setSlewRateInitial(VectorXd::Constant(m, 1.0));
  if (opts.slew_control_points)
    mpc.setSlewRate(VectorXd::Constant(m, 1.0));
  if (opts.state_bounds != ampc::StateBoundMode::Off) {
    const auto k{static_cast<Index>(mpc.getBoundedStates().size())};
    mpc.setStateLimits(VectorXd::Constant(k, -10), VectorXd::Constant(k, 10));
  }
  if (opts.state_bounds == ampc::StateBoundMode::Soft) {
    const auto k{static_cast<Index>(mpc.getBoundedStates().size())};
    mpc.setStateBoundPenalties(VectorXd::Constant(k, 10.0),
                               VectorXd::Constant(k, 100.0));
  }
  // x + 0.1 v + 0.5 u <= 5 at every step, x + v <= 5 at the end
  const RowVector2d Hx{1.0, 0.1}, Hf{1.0, 1.0};
  const VectorXd Hu{VectorXd::Constant(1, 0.5)};
  const VectorXd h_min{VectorXd::Constant(1, -5.0)};
  const VectorXd h_max{VectorXd::Constant(1, 5.0)};
  if (opts.num_general_constraints > 0)
    mpc.setGeneralConstraints(Hx, Hu, h_min, h_max);
  if (opts.num_terminal_constraints > 0)
    mpc.setTerminalConstraints(Hf, h_min, h_max);
  ASSERT_TRUE(mpc.initializeSolver());

  const Vector2d x0{0.5, -0.2};
  VectorXd u0{m}, u_traj{m * param.horizon_steps};
  VectorXd x_traj{n * param.horizon_steps};
  auto solve = [&]() {
    if (use_prepare) {
      // counted separately below, so only solve() is counted here
      counting = false;
      EXPECT_EQ(mpc.prepare(), ampc::SolveStatus::Success);
      counting = true;
    }
    const ampc::SolveStatus status{mpc.solve(x0)};
    mpc.getNextInput(u0);
    mpc.getInputTrajectory(u_traj);
    mpc.getPredictedStateTrajectory(x_traj);
    return status;
  };

  // the first solve applies the updates pending since initializeSolver()
  ampc::SolveStatus status{};
  EXPECT_EQ(countAllocations([&] { status = solve(); }), 0u);
  ASSERT_EQ(status, ampc::SolveStatus::Success);
  EXPECT_EQ(countAllocations([&] { status = solve(); }), 0u);
  ASSERT_EQ(status, ampc::SolveStatus::Success);

  // each change category rebuilds a different part of the QP in prepare() or
  // solve()
  if (use_prepare) {
    mpc.setStateWeights(Vector2d{1.5, 0.15});
    mpc.setModelOffsetDiscrete(Vector2d{0.0, 0.01});
    EXPECT_EQ(countAllocations([&] { (void)mpc.prepare(); }), 0u) << "prepare";
  }
  mpc.setStateWeights(Vector2d{2.0, 0.2});
  EXPECT_EQ(countAllocations([&] { status = solve(); }), 0u) << "weights";
  mpc.setModelContinuous2Discrete(1.1 * Ac, Bc, Vector2d{0.0, 0.01}, 0.1);
  EXPECT_EQ(countAllocations([&] { status = solve(); }), 0u) << "model";
  mpc.setModelOffsetDiscrete(Vector2d{0.0, 0.02});
  EXPECT_EQ(countAllocations([&] { status = solve(); }), 0u) << "offset";
  mpc.setReferenceState(Vector2d{0.5, 0.0});
  EXPECT_EQ(countAllocations([&] { status = solve(); }), 0u) << "references";
  mpc.setInputLimits(VectorXd::Constant(m, -2.0), VectorXd::Constant(m, 2.0));
  EXPECT_EQ(countAllocations([&] { status = solve(); }), 0u) << "limits";
  if (opts.num_general_constraints > 0) {
    mpc.setGeneralConstraints(1.1 * Hx, Hu, h_min, h_max);
    EXPECT_EQ(countAllocations([&] { status = solve(); }), 0u)
        << "constraint matrices";
  }
  if (opts.num_terminal_constraints > 0) {
    mpc.setTerminalConstraints(Hf, 2.0 * h_min, 2.0 * h_max);
    EXPECT_EQ(countAllocations([&] { status = solve(); }), 0u)
        << "constraint limits";
  }
  ASSERT_EQ(status, ampc::SolveStatus::Success);
}

struct AllocationCase
{
  std::string name;
  ampc::Parameterization param;
  ampc::Options opts;
};

// Small problems only: for large condensed problems, Eigen's matrix product
// heap-allocates blocking buffers when P is rebuilt after a model or weight
// change, in prepare() or solve() (steady-state solves still do not allocate).
std::vector<AllocationCase> allocationCases()
{
  const ampc::Options all{.use_input_cost = true,
                          .use_input_rate_cost = true,
                          .slew_initial_input = true,
                          .slew_control_points = true,
                          .state_bounds = ampc::StateBoundMode::Hard};
  const ampc::Options sampled{.use_input_cost = true,
                              .saturate_input_trajectory = true};
  const ampc::Options soft{.use_input_cost = true,
                           .state_bounds = ampc::StateBoundMode::Soft,
                           .bounded_states = {0}};
  const ampc::Options constraints{.use_input_cost = true,
                                  .num_general_constraints = 1,
                                  .num_terminal_constraints = 1};
  return {
      {"defaults", ampc::Parameterization::linearInterp(10, 4), {}},
      {"allOptions", ampc::Parameterization::linearInterp(10, 4), all},
      {"sampledBounds", ampc::Parameterization::bspline(10, 2, 5), sampled},
      {"moveBlocking", ampc::Parameterization::moveBlocking(10, 10), all},
      {"softPosition", ampc::Parameterization::bspline(10, 2, 5), soft},
      {"generalConstraints", ampc::Parameterization::bspline(10, 2, 5),
       constraints},
  };
}

class AllocationTester : public ::testing::TestWithParam<AllocationCase>
{};

TEST_P(AllocationTester, givenInitializedCondensedMPC_SolveDoesNotAllocate)
{
  checkNoAllocationsAfterInitialize<ampc::Formulation::Condensed>(
      GetParam().param, GetParam().opts, false);
}

TEST_P(AllocationTester, givenInitializedSparseMPC_SolveDoesNotAllocate)
{
  checkNoAllocationsAfterInitialize<ampc::Formulation::Sparse>(
      GetParam().param, GetParam().opts, false);
}

TEST_P(AllocationTester,
       givenInitializedCondensedMPC_PrepareAndSolveDoNotAllocate)
{
  checkNoAllocationsAfterInitialize<ampc::Formulation::Condensed>(
      GetParam().param, GetParam().opts, true);
}

TEST_P(AllocationTester, givenInitializedSparseMPC_PrepareAndSolveDoNotAllocate)
{
  checkNoAllocationsAfterInitialize<ampc::Formulation::Sparse>(
      GetParam().param, GetParam().opts, true);
}

TEST(AllocationTester, givenLargeCondensedMPCAfterPrepare_SolveDoesNotAllocate)
{
#ifndef AMPC_COUNT_ALLOCATIONS
  GTEST_SKIP() << "allocation counting needs glibc and no sanitizer";
#endif
  // Large enough that Eigen's matrix product for P may heap-allocate its
  // blocking buffers. prepare() does that work, so solve() still does not
  // allocate.
  const int n{4}, m{2}, T{300}, nc{60};
  const ampc::Options opts{.use_input_cost = true};
  ampc::MPC mpc{n, m, ampc::Parameterization::bspline(T, 2, nc), opts};
  MatrixXd Ad{MatrixXd::Identity(n, n)};
  Ad.topRightCorner(n - 1, n - 1).diagonal().setConstant(0.01);
  MatrixXd Bd{MatrixXd::Zero(n, m)};
  Bd.bottomRows(m).setConstant(0.01);
  Bd(0, 0) = 0.001;
  mpc.setModelDiscrete(Ad, Bd, VectorXd::Zero(n));
  mpc.setInputLimits(VectorXd::Constant(m, -1.0), VectorXd::Constant(m, 1.0));
  mpc.setReferenceState(VectorXd::Ones(n));
  ASSERT_TRUE(mpc.initializeSolver());
  const VectorXd x0{VectorXd::Zero(n)};
  ASSERT_EQ(mpc.solve(x0), ampc::SolveStatus::Success);

  mpc.setModelDiscrete(1.001 * Ad, Bd, VectorXd::Zero(n));
  ASSERT_EQ(mpc.prepare(), ampc::SolveStatus::Success);
  ampc::SolveStatus status{};
  EXPECT_EQ(countAllocations([&] { status = mpc.solve(x0); }), 0u);
  EXPECT_EQ(status, ampc::SolveStatus::Success);
}

INSTANTIATE_TEST_SUITE_P(
    Cases,
    AllocationTester,
    ::testing::ValuesIn(allocationCases()),
    [](const ::testing::TestParamInfo<AllocationCase>& info) {
      return info.param.name;
    });

TEST(AllocationCounter, givenHeapAllocation_CountsIt)
{
#ifndef AMPC_COUNT_ALLOCATIONS
  GTEST_SKIP() << "allocation counting needs glibc and no sanitizer";
#endif
  // guards against the interposition silently not taking effect
  const std::size_t count{countAllocations([] {
    VectorXd v{1000};
    v.setZero();
  })};
  EXPECT_GE(count, 1u);
}

} // namespace
