#include "affine_mpc/mpc_logger.hpp"

#include <filesystem>
#include <fstream>
#include <gtest/gtest.h>
#include <iterator>
#include <stdexcept>
#include <string>
#include <tuple>
#include <utility>

#include "affine_mpc/mpc.hpp"
#include "utils.hpp"

namespace ampc = affine_mpc;
namespace fs = std::filesystem;

class MPCLoggerTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    test_dir_ = uniqueTempDir("mpc_logger_test");
    if (fs::exists(test_dir_)) {
      fs::remove_all(test_dir_);
    }

    const int n{2}, m{1}, T{5}, p{2};
    const auto param{ampc::Parameterization::linearInterp(T, p)};
    const ampc::Options opts{.input_cost = ampc::InputCostMode::ControlPoints};
    mpc_ = std::make_unique<ampc::MPC>(n, m, param, opts);

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
  std::unique_ptr<ampc::MPC> mpc_;
};

TEST_F(MPCLoggerTest, ConstructorCreatesDirectoryAndTempFiles)
{
  ampc::MPCLogger logger(mpc_.get(), test_dir_, 0.1, 1, false, "test_log");

  EXPECT_TRUE(fs::exists(test_dir_));
  EXPECT_TRUE(fs::is_directory(test_dir_));

  const fs::path raw_dir = test_dir_ / "test_log_raw";
  EXPECT_TRUE(fs::exists(raw_dir));
  EXPECT_TRUE(fs::exists(raw_dir / "time.bin"));
  EXPECT_TRUE(fs::exists(raw_dir / "states.bin"));
  EXPECT_TRUE(fs::exists(raw_dir / "inputs.bin"));
  EXPECT_TRUE(fs::exists(raw_dir / "loop_time.bin"));
  EXPECT_TRUE(fs::exists(raw_dir / "mpc_solve_time.bin"));
  EXPECT_TRUE(fs::exists(raw_dir / "qp_solve_time.bin"));
  EXPECT_TRUE(fs::exists(raw_dir / "prepare_time.bin"));
}

TEST_F(MPCLoggerTest, ConvenienceLogStepWorks)
{
  ampc::MPCLogger logger(mpc_.get(), test_dir_, 0.1, 1, false, "conv_log");

  Eigen::Vector2d x0{1.0, 0.5};
  ASSERT_EQ(mpc_->solve(x0), ampc::SolveStatus::Success);
  logger.logStep(0.0, x0, 0.001);
  logger.finalize();

  EXPECT_TRUE(fs::exists(test_dir_ / "conv_log.npz"));
  EXPECT_FALSE(fs::exists(test_dir_ / "conv_log_raw"));
}

TEST_F(MPCLoggerTest, RawRecoverableModeWritesRecoverableOutputs)
{
  ampc::MPCLogger logger(mpc_.get(), test_dir_, 0.1, 1, false, "raw_log",
                         ampc::MPCLogger::Mode::RawRecoverable);

  Eigen::Vector2d x0{1.0, 0.5};
  ASSERT_EQ(mpc_->solve(x0), ampc::SolveStatus::Success);
  logger.logStep(0.0, x0, 0.001);
  logger.finalize();

  const fs::path raw_dir = test_dir_ / "raw_log_raw";
  EXPECT_TRUE(fs::exists(raw_dir / "states.bin"));
  EXPECT_TRUE(fs::exists(raw_dir / "states.npyh"));
  EXPECT_TRUE(fs::exists(raw_dir / "params.yaml"));
  EXPECT_TRUE(fs::exists(raw_dir / "data_info.yaml"));
  EXPECT_FALSE(fs::exists(test_dir_ / "raw_log.npz"));
}

TEST_F(MPCLoggerTest, NpyModeWritesNpyDirectory)
{
  ampc::MPCLogger logger(mpc_.get(), test_dir_, 0.1, 1, false, "npy_log",
                         ampc::MPCLogger::Mode::Npy);

  Eigen::Vector2d x0{1.0, 0.5};
  ASSERT_EQ(mpc_->solve(x0), ampc::SolveStatus::Success);
  logger.logStep(0.0, x0, 0.001);
  logger.finalize();

  EXPECT_TRUE(fs::exists(test_dir_ / "npy_log_npy" / "states.npy"));
  EXPECT_TRUE(fs::exists(test_dir_ / "params.yaml"));
  EXPECT_FALSE(fs::exists(test_dir_ / "npy_log_raw"));
}

TEST_F(MPCLoggerTest, NpzUncompressedModeWritesArchive)
{
  ampc::MPCLogger logger(mpc_.get(), test_dir_, 0.1, 1, false, "stored_log",
                         ampc::MPCLogger::Mode::NpzUncompressed);

  Eigen::Vector2d x0{1.0, 0.5};
  ASSERT_EQ(mpc_->solve(x0), ampc::SolveStatus::Success);
  logger.logStep(0.0, x0, 0.001);
  logger.finalize();

  EXPECT_TRUE(fs::exists(test_dir_ / "stored_log.npz"));
  EXPECT_FALSE(fs::exists(test_dir_ / "stored_log_raw"));
}

TEST(MPCLoggerNoInputCostTest, ConvenienceLogStepWorksWithoutInputCost)
{
  const fs::path test_dir{uniqueTempDir("mpc_logger_no_input_cost")};
  if (fs::exists(test_dir)) {
    fs::remove_all(test_dir);
  }

  const int n{2}, m{1}, T{5}, p{2};
  const auto param{ampc::Parameterization::linearInterp(T, p)};
  ampc::MPC mpc{n, m, param};

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

TEST(MPCLoggerUninitializedTest, LogStepBeforeInitializeSolver_Throws)
{
  const fs::path test_dir{uniqueTempDir("mpc_logger_uninitialized")};
  if (fs::exists(test_dir)) {
    fs::remove_all(test_dir);
  }

  const int n{2}, m{1}, T{5}, p{2};
  const auto param{ampc::Parameterization::linearInterp(T, p)};
  ampc::MPC mpc{n, m, param};

  {
    ampc::MPCLogger logger{&mpc, test_dir, 0.1, 1, false, "uninitialized"};
    EXPECT_THROW(logger.logStep(0.0, Eigen::Vector2d{1.0, 0.5}),
                 std::logic_error);
  }

  fs::remove_all(test_dir);
}

TEST_F(MPCLoggerTest, LogStepWithWrongSizeState_Throws)
{
  ampc::MPCLogger logger(mpc_.get(), test_dir_, 0.1, 1, false, "bad_x0");

  const Eigen::Vector2d x0{1.0, 0.5};
  ASSERT_EQ(mpc_->solve(x0), ampc::SolveStatus::Success);
  EXPECT_THROW(logger.logStep(0.0, Eigen::Vector3d{1.0, 0.5, 0.0}),
               std::invalid_argument);
}

TEST(MPCLoggerFormulationTest, givenEachFormulation_RecordsItInParams)
{
  const fs::path test_dir{uniqueTempDir("mpc_logger_formulation")};
  const auto param{ampc::Parameterization::linearInterp(5, 2)};
  for (const auto& [formulation, name] :
       {std::pair{ampc::Formulation::Condensed, "condensed"},
        std::pair{ampc::Formulation::Sparse, "sparse"}}) {
    SCOPED_TRACE(name);
    fs::remove_all(test_dir);
    ampc::MPC mpc{2, 1, param, formulation};
    {
      ampc::MPCLogger logger{&mpc, test_dir, 0.1};
      logger.finalize();
    }
    std::ifstream file{test_dir / "params.yaml"};
    const std::string params{std::istreambuf_iterator<char>{file}, {}};
    EXPECT_NE(params.find(std::string{"formulation: "} + name + "\n"),
              std::string::npos)
        << params;
  }
  fs::remove_all(test_dir);
}

TEST(MPCLoggerOptionsTest, givenStateBoundMode_RecordsModeAndReleasedKey)
{
  const fs::path test_dir{uniqueTempDir("mpc_logger_state_bounds")};
  const auto param{ampc::Parameterization::linearInterp(5, 2)};
  for (const auto& [mode, name, saturate] :
       {std::tuple{ampc::StateBoundMode::Off, "off", 0},
        std::tuple{ampc::StateBoundMode::Hard, "hard", 1},
        std::tuple{ampc::StateBoundMode::Soft, "soft", 1}}) {
    SCOPED_TRACE(name);
    fs::remove_all(test_dir);
    ampc::MPC mpc{2, 1, param, ampc::Options{.state_bounds = mode}};
    {
      ampc::MPCLogger logger{&mpc, test_dir, 0.1};
      logger.finalize();
    }
    std::ifstream file{test_dir / "params.yaml"};
    const std::string params{std::istreambuf_iterator<char>{file}, {}};
    EXPECT_NE(params.find(std::string{"opt_state_bounds: "} + name + "\n"),
              std::string::npos)
        << params;
    EXPECT_NE(
        params.find("opt_saturate_states: " + std::to_string(saturate) + "\n"),
        std::string::npos)
        << params;
    EXPECT_EQ(params.find("opt_soft_state_bounds"), std::string::npos);
  }
  fs::remove_all(test_dir);
}

TEST(MPCLoggerOptionsTest, givenInputBoundMode_RecordsModeAndReleasedKey)
{
  const fs::path test_dir{uniqueTempDir("mpc_logger_input_bounds")};
  const auto param{ampc::Parameterization::bspline(5, 2, 3)};
  for (const auto& [mode, name, saturate] :
       {std::tuple{ampc::InputBoundMode::ControlPoints, "control_points", 0},
        std::tuple{ampc::InputBoundMode::Exact, "exact", 1},
        std::tuple{ampc::InputBoundMode::Refined, "refined", 0}}) {
    SCOPED_TRACE(name);
    fs::remove_all(test_dir);
    ampc::MPC mpc{
        2, 1, param,
        ampc::Options{.input_bounds = mode, .refined_bound_interval = 3}};
    {
      ampc::MPCLogger logger{&mpc, test_dir, 0.1};
      logger.finalize();
    }
    std::ifstream file{test_dir / "params.yaml"};
    const std::string params{std::istreambuf_iterator<char>{file}, {}};
    EXPECT_NE(params.find(std::string{"opt_input_bounds: "} + name + "\n"),
              std::string::npos)
        << params;
    EXPECT_NE(params.find("opt_saturate_input_trajectory: "
                          + std::to_string(saturate) + "\n"),
              std::string::npos)
        << params;
    EXPECT_NE(params.find("opt_refined_bound_interval: 3\n"), std::string::npos)
        << params;
  }
  fs::remove_all(test_dir);
}

TEST(MPCLoggerOptionsTest, givenInputCostMode_RecordsModeAndReleasedKey)
{
  const fs::path test_dir{uniqueTempDir("mpc_logger_input_cost")};
  const auto param{ampc::Parameterization::bspline(5, 2, 3)};
  for (const auto& [mode, name, use_input_cost] :
       {std::tuple{ampc::InputCostMode::Off, "off", 0},
        std::tuple{ampc::InputCostMode::ControlPoints, "control_points", 1},
        std::tuple{ampc::InputCostMode::Trajectory, "trajectory", 1}}) {
    SCOPED_TRACE(name);
    fs::remove_all(test_dir);
    ampc::MPC mpc{2, 1, param, ampc::Options{.input_cost = mode}};
    {
      ampc::MPCLogger logger{&mpc, test_dir, 0.1};
      logger.finalize();
    }
    std::ifstream file{test_dir / "params.yaml"};
    const std::string params{std::istreambuf_iterator<char>{file}, {}};
    EXPECT_NE(params.find(std::string{"opt_input_cost: "} + name + "\n"),
              std::string::npos)
        << params;
    EXPECT_NE(params.find("opt_use_input_cost: "
                          + std::to_string(use_input_cost) + "\n"),
              std::string::npos)
        << params;
    EXPECT_EQ(params.find("R_diag") != std::string::npos, use_input_cost == 1)
        << params;
  }
  fs::remove_all(test_dir);
}

TEST(MPCLoggerOptionsTest, givenTrajectoryInputCost_LogsSampledReferenceOnly)
{
  // the sampled reference is logged with the sampled inputs, and not with
  // control points (there is no control point reference)
  const fs::path test_dir{uniqueTempDir("mpc_logger_trajectory_ref")};
  const int n{2}, m{1}, T{5};
  const auto param{ampc::Parameterization::linearInterp(T, 3)};
  for (const bool log_control_points : {false, true}) {
    SCOPED_TRACE(log_control_points ? "control points" : "inputs");
    fs::remove_all(test_dir);
    ampc::MPC mpc{n, m, param,
                  ampc::Options{.input_cost = ampc::InputCostMode::Trajectory}};
    Eigen::Matrix2d A;
    A << 1, 0.1, 0, 1;
    mpc.setModelContinuous2Discrete(A, Eigen::Vector2d{0, 0.1},
                                    Eigen::Vector2d::Zero(), 0.1);
    mpc.setInputLimits(Eigen::VectorXd::Constant(m, -1),
                       Eigen::VectorXd::Constant(m, 1));
    mpc.setReferenceInputTrajectory(Eigen::VectorXd::LinSpaced(T, 0.0, 0.4));
    ASSERT_TRUE(mpc.initializeSolver());
    {
      ampc::MPCLogger logger{&mpc,
                             test_dir,
                             0.1,
                             1,
                             log_control_points,
                             "log",
                             ampc::MPCLogger::Mode::Npy};
      const Eigen::Vector2d x0{1.0, 0.5};
      ASSERT_EQ(mpc.solve(x0), ampc::SolveStatus::Success);
      logger.logStep(0.0, x0, 0.001);
      logger.finalize();
    }
    EXPECT_TRUE(fs::exists(test_dir / "log_npy" / "inputs.npy"));
    // the sampled reference is logged at the strided steps, like ref_states,
    // also with log_control_points
    EXPECT_TRUE(fs::exists(test_dir / "log_npy" / "ref_inputs.npy"));
  }
  fs::remove_all(test_dir);
}
