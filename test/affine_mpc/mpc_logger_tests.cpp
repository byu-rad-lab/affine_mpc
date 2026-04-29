#include "affine_mpc/mpc_logger.hpp"

#include <filesystem>
#include <gtest/gtest.h>

#include "affine_mpc/condensed_mpc.hpp"

namespace ampc = affine_mpc;
namespace fs = std::filesystem;

class MPCLoggerTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    test_dir_ = fs::temp_directory_path() / "mpc_logger_test";
    if (fs::exists(test_dir_)) {
      fs::remove_all(test_dir_);
    }

    const int n{2}, m{1}, T{5}, p{2};
    const auto param{ampc::Parameterization::linearInterp(T, p)};
    const ampc::Options opts{.use_input_cost = true};
    mpc_ = std::make_unique<ampc::CondensedMPC>(n, m, param, opts);

    Eigen::Matrix2d A;
    A << 1, 0.1, 0, 1;
    Eigen::Vector2d B{0, 0.1}, w{0, 0};
    mpc_->setModelContinuous2Discrete(A, B, w, 0.1);
    mpc_->setInputLimits(Eigen::VectorXd::Constant(m, -1),
                         Eigen::VectorXd::Constant(m, 1));
    mpc_->setWeights(Eigen::VectorXd::Ones(n), Eigen::VectorXd::Ones(m));
    mpc_->setReferenceState(Eigen::VectorXd::Zero(n));

    ASSERT_TRUE(mpc_->initializeSolver());
  }

  void TearDown() override
  {
    if (fs::exists(test_dir_)) {
      fs::remove_all(test_dir_);
    }
  }

  fs::path test_dir_;
  std::unique_ptr<ampc::CondensedMPC> mpc_;
};

TEST_F(MPCLoggerTest, ConstructorCreatesDirectoryAndTempFiles)
{
  ampc::MPCLogger logger(mpc_.get(), test_dir_, 0.1, 1, false, "test_log");

  EXPECT_TRUE(fs::exists(test_dir_));
  EXPECT_TRUE(fs::is_directory(test_dir_));

  EXPECT_TRUE(fs::exists(test_dir_ / "test_log_time.tmp"));
  EXPECT_TRUE(fs::exists(test_dir_ / "test_log_states.tmp"));
  EXPECT_TRUE(fs::exists(test_dir_ / "test_log_inputs.tmp"));
  EXPECT_TRUE(fs::exists(test_dir_ / "test_log_solve_times.tmp"));
}

TEST_F(MPCLoggerTest, ConvenienceLogStepWorks)
{
  ampc::MPCLogger logger(mpc_.get(), test_dir_, 0.1, 1, false, "conv_log");

  Eigen::Vector2d x0{1.0, 0.5};
  const auto status = mpc_->solve(x0);
  logger.logStep(0.0, x0, 0.001);
  logger.finalize();

  EXPECT_TRUE(fs::exists(test_dir_ / "conv_log.npz"));
}

TEST(MPCLoggerNoInputCostTest, ConvenienceLogStepWorksWithoutInputCost)
{
  const fs::path test_dir =
      fs::temp_directory_path() / "mpc_logger_no_input_cost";
  if (fs::exists(test_dir)) {
    fs::remove_all(test_dir);
  }

  const int n{2}, m{1}, T{5}, p{2};
  const auto param{ampc::Parameterization::linearInterp(T, p)};
  ampc::CondensedMPC mpc{n, m, param};

  Eigen::Matrix2d A;
  A << 1, 0.1, 0, 1;
  Eigen::Vector2d B{0, 0.1}, w{0, 0};
  mpc.setModelContinuous2Discrete(A, B, w, 0.1);
  mpc.setInputLimits(Eigen::VectorXd::Constant(m, -1),
                     Eigen::VectorXd::Constant(m, 1));
  mpc.setStateWeights(Eigen::VectorXd::Ones(n));
  mpc.setReferenceState(Eigen::VectorXd::Zero(n));
  ASSERT_TRUE(mpc.initializeSolver());

  ampc::MPCLogger logger{&mpc, test_dir, 0.1, 1, false, "no_input_cost"};

  Eigen::Vector2d x0{1.0, 0.5};
  ASSERT_EQ(mpc.solve(x0), ampc::SolveStatus::Success);
  EXPECT_NO_THROW(logger.logStep(0.0, x0, 0.001));
  logger.finalize();

  EXPECT_TRUE(fs::exists(test_dir / "no_input_cost.npz"));

  if (fs::exists(test_dir)) {
    fs::remove_all(test_dir);
  }
}
