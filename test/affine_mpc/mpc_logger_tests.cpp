#include <gtest/gtest.h>
#include <filesystem>
#include <cnpy.h>
#include "affine_mpc/mpc_logger.hpp"
#include "affine_mpc/condensed_mpc.hpp"
#include "utils.hpp"

namespace ampc = affine_mpc;
namespace fs = std::filesystem;

class MPCLoggerTest : public ::testing::Test {
protected:
    void SetUp() override {
        test_dir_ = fs::temp_directory_path() / "mpc_logger_test";
        if (fs::exists(test_dir_)) {
            fs::remove_all(test_dir_);
        }
        
        // Setup a simple MPC instance
        const int n{2}, m{1}, T{5}, p{2};
        const auto param{ampc::Parameterization::linearInterp(T, p)};
        const ampc::Options opts{.use_input_cost = true};
        mpc_ = std::make_unique<ampc::CondensedMPC>(n, m, param, opts);
        
        Eigen::Matrix2d A;
        A << 1, 0.1, 0, 1;
        Eigen::Vector2d B{0, 0.1}, w{0, 0};
        mpc_->setModelContinuous2Discrete(A, B, w, 0.1);
        mpc_->setInputLimits(Eigen::VectorXd::Constant(m, -1), Eigen::VectorXd::Constant(m, 1));
        mpc_->setWeights(Eigen::VectorXd::Ones(n), Eigen::VectorXd::Ones(m));
        mpc_->setReferenceState(Eigen::VectorXd::Zero(n));
        
        ASSERT_TRUE(mpc_->initializeSolver());
    }

    void TearDown() override {
        if (fs::exists(test_dir_)) {
            fs::remove_all(test_dir_);
        }
    }

    fs::path test_dir_;
    std::unique_ptr<ampc::CondensedMPC> mpc_;
};

TEST_F(MPCLoggerTest, ConstructorCreatesDirectoryAndTempFiles) {
    ampc::MPCLogger logger(mpc_.get(), test_dir_, "test_log");
    
    EXPECT_TRUE(fs::exists(test_dir_));
    EXPECT_TRUE(fs::is_directory(test_dir_));
    
    // Check if temp files exist (they should be created in the constructor)
    EXPECT_TRUE(fs::exists(test_dir_ / "test_log_time.tmp"));
    EXPECT_TRUE(fs::exists(test_dir_ / "test_log_states.tmp"));
    EXPECT_TRUE(fs::exists(test_dir_ / "test_log_refs.tmp"));
    EXPECT_TRUE(fs::exists(test_dir_ / "test_log_inputs.tmp"));
    EXPECT_TRUE(fs::exists(test_dir_ / "test_log_solve_times.tmp"));
}

TEST_F(MPCLoggerTest, FinalizeCreatesNpzAndCleansUp) {
    {
        ampc::MPCLogger logger(mpc_.get(), test_dir_, "test_log");
        
        Eigen::Vector2d x0{1.0, 0.5};
        logger.logPreviousSolve(0.0, 0.1, x0, 0.001);
        logger.logPreviousSolve(0.1, 0.1, x0, 0.001);
        
        logger.finalize();
        
        EXPECT_TRUE(fs::exists(test_dir_ / "test_log.npz"));
        EXPECT_TRUE(fs::exists(test_dir_ / "params.yaml"));
        
        // Temp files should be gone
        EXPECT_FALSE(fs::exists(test_dir_ / "test_log_time.tmp"));
        EXPECT_FALSE(fs::exists(test_dir_ / "test_log_states.tmp"));
    }
}

TEST_F(MPCLoggerTest, LoggedDataIsCorrect) {
    ampc::MPCLogger logger(mpc_.get(), test_dir_, "test_log");
    
    Eigen::Vector2d x0{1.0, 0.5};
    logger.logPreviousSolve(0.0, 0.1, x0, 0.005);
    logger.finalize();
    
    cnpy::npz_t my_npz = cnpy::npz_load((test_dir_ / "test_log.npz").string());
    
    // Check time
    auto it_time = my_npz.find("time");
    ASSERT_NE(it_time, my_npz.end());
    EXPECT_EQ(it_time->second.shape.size(), 1);
    EXPECT_EQ(it_time->second.shape[0], 1);
    EXPECT_EQ(it_time->second.data<double>()[0], 0.0);
    
    // Check shapes
    // x_pred should be (N, T+1, n) = (1, 6, 2)
    auto it_x = my_npz.find("x_pred");
    ASSERT_NE(it_x, my_npz.end());
    ASSERT_EQ(it_x->second.shape.size(), 3);
    EXPECT_EQ(it_x->second.shape[0], 1);
    EXPECT_EQ(it_x->second.shape[1], 6);
    EXPECT_EQ(it_x->second.shape[2], 2);
    
    // u_pred should be (N, T, m) = (1, 5, 1)
    auto it_u = my_npz.find("u_pred");
    ASSERT_NE(it_u, my_npz.end());
    ASSERT_EQ(it_u->second.shape.size(), 3);
    EXPECT_EQ(it_u->second.shape[0], 1);
    EXPECT_EQ(it_u->second.shape[1], 5);
    EXPECT_EQ(it_u->second.shape[2], 1);
}
