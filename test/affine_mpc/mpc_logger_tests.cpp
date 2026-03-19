#include "affine_mpc/mpc_logger.hpp"

#include <cnpy.h>
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
  ampc::MPCLogger logger(*mpc_, test_dir_, 0.1, 1, false, "test_log");

  EXPECT_TRUE(fs::exists(test_dir_));
  EXPECT_TRUE(fs::is_directory(test_dir_));

  EXPECT_TRUE(fs::exists(test_dir_ / "test_log_time.tmp"));
  EXPECT_TRUE(fs::exists(test_dir_ / "test_log_states.tmp"));
  EXPECT_TRUE(fs::exists(test_dir_ / "test_log_inputs.tmp"));
  EXPECT_TRUE(fs::exists(test_dir_ / "test_log_solve_times.tmp"));
}

TEST_F(MPCLoggerTest, ConvenienceLogStepWorks)
{
  ampc::MPCLogger logger(*mpc_, test_dir_, 0.1, 1, false, "conv_log");

  Eigen::Vector2d x0{1.0, 0.5};
  const auto status = mpc_->solve(x0);
  logger.logStep(0.0, x0, *mpc_, 0.001);
  logger.finalize();

  EXPECT_TRUE(fs::exists(test_dir_ / "conv_log.npz"));
  cnpy::npz_t my_npz = cnpy::npz_load((test_dir_ / "conv_log.npz").string());

  EXPECT_EQ(my_npz["states"].shape[0], 1); // N
  EXPECT_EQ(my_npz["states"].shape[1], 6); // K = T+1 (stride=1)
  EXPECT_EQ(my_npz["states"].shape[2], 2); // n

  EXPECT_EQ(my_npz["inputs"].shape[0], 1); // N
  EXPECT_EQ(my_npz["inputs"].shape[1], 6); // K = T+1 (stride=1)
  EXPECT_EQ(my_npz["inputs"].shape[2], 1); // m

  // Check t_pred
  ASSERT_NE(my_npz.find("meta_t_pred"), my_npz.end());
  EXPECT_EQ(my_npz["meta_t_pred"].shape[0], 6);
  EXPECT_DOUBLE_EQ(my_npz["meta_t_pred"].data<double>()[5], 0.5); // T*ts
}

TEST_F(MPCLoggerTest, StrideWorksAsExpected)
{
  ampc::MPCLogger logger(*mpc_, test_dir_, 0.1, 2, false,
                         "stride_log"); // stride=2

  Eigen::Vector2d x0{1.0, 0.5};
  const auto status = mpc_->solve(x0);
  logger.logStep(0.0, x0, *mpc_, 0.001);
  logger.finalize();

  cnpy::npz_t my_npz = cnpy::npz_load((test_dir_ / "stride_log.npz").string());

  // T=5, stride=2 -> k = [0, 2, 4, 5] -> K=4
  EXPECT_EQ(my_npz["states"].shape[1], 4);
  EXPECT_EQ(my_npz["inputs"].shape[1], 4);

  EXPECT_DOUBLE_EQ(my_npz["meta_t_pred"].data<double>()[0], 0.0);
  EXPECT_DOUBLE_EQ(my_npz["meta_t_pred"].data<double>()[1], 0.2);
  EXPECT_DOUBLE_EQ(my_npz["meta_t_pred"].data<double>()[2], 0.4);
  EXPECT_DOUBLE_EQ(my_npz["meta_t_pred"].data<double>()[3], 0.5);
}

TEST_F(MPCLoggerTest, ControlPointsLoggingWorks)
{
  ampc::MPCLogger logger(*mpc_, test_dir_, 0.1, 1, true,
                         "ctrl_log"); // log_control_points=true

  Eigen::Vector2d x0{1.0, 0.5};
  const auto status = mpc_->solve(x0);
  logger.logStep(0.0, x0, *mpc_, 0.001);
  logger.finalize();

  cnpy::npz_t my_npz = cnpy::npz_load((test_dir_ / "ctrl_log.npz").string());

  // states should still be K=6
  EXPECT_EQ(my_npz["states"].shape[1], 6);

  // inputs should be num_control_points (p=2 -> nc=2 for linearInterp with T=5?
  // No, linearInterp(T=5, p=2) means nc=2. Wait, linearInterp(horizon,
  // change_points) so change_points=2) mpc_->num_ctrl_pts_ is the actual value.
  // Let's just check it matches the metadata.
  int nc = my_npz["meta_num_control_points"].data<int>()[0];
  EXPECT_EQ(my_npz["inputs"].shape[1], nc);
}

TEST_F(MPCLoggerTest, ZeroStrideSqueezesArrays)
{
  ampc::MPCLogger logger(*mpc_, test_dir_, 0.1, 0, false,
                         "squeeze_log"); // stride=0

  Eigen::Vector2d x0{1.0, 0.5};
  const auto status = mpc_->solve(x0);
  logger.logStep(0.0, x0, *mpc_, 0.001);
  logger.finalize();

  cnpy::npz_t my_npz = cnpy::npz_load((test_dir_ / "squeeze_log.npz").string());

  // states and inputs should be 2D: (N, n) and (N, m)
  EXPECT_EQ(my_npz["states"].shape.size(), 2);
  EXPECT_EQ(my_npz["states"].shape[0], 1); // N
  EXPECT_EQ(my_npz["states"].shape[1], 2); // n

  EXPECT_EQ(my_npz["inputs"].shape.size(), 2);
  EXPECT_EQ(my_npz["inputs"].shape[0], 1); // N
  EXPECT_EQ(my_npz["inputs"].shape[1], 1); // m
}
