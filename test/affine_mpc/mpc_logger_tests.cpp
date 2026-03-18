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
    
    EXPECT_TRUE(fs::exists(test_dir_ / "test_log_time.tmp"));
    EXPECT_TRUE(fs::exists(test_dir_ / "test_log_states.tmp"));
    EXPECT_TRUE(fs::exists(test_dir_ / "test_log_inputs.tmp"));
    EXPECT_TRUE(fs::exists(test_dir_ / "test_log_solve_times.tmp"));
}

TEST_F(MPCLoggerTest, GenericConstructorWorks) {
    ampc::MPCLogger logger(2, 1, 10, test_dir_, "generic_log");
    logger.addMetadata("custom_val", 42);
    logger.finalize();
    
    EXPECT_TRUE(fs::exists(test_dir_ / "generic_log.npz"));
    cnpy::npz_t my_npz = cnpy::npz_load((test_dir_ / "generic_log.npz").string());
    ASSERT_NE(my_npz.find("meta_custom_val"), my_npz.end());
    EXPECT_EQ(my_npz["meta_custom_val"].data<int>()[0], 42);
}

TEST_F(MPCLoggerTest, MetadataPrecisionIsRespectedInYaml) {
    ampc::MPCLogger logger(2, 1, 10, test_dir_, "prec_log");
    logger.addMetadata("high_prec", 3.1415926535, 8);
    logger.addMetadata("low_prec", 3.1415926535, 2);
    logger.finalize();
    
    std::ifstream fin(test_dir_ / "params.yaml");
    std::string line;
    bool found_high = false, found_low = false;
    while (std::getline(fin, line)) {
        if (line.find("high_prec: 3.14159265") != std::string::npos) found_high = true;
        if (line.find("low_prec: 3.14") != std::string::npos && line.find("3.141") == std::string::npos) found_low = true;
    }
    EXPECT_TRUE(found_high);
    EXPECT_TRUE(found_low);
}

TEST_F(MPCLoggerTest, LoggedDataIsCorrect) {
    ampc::MPCLogger logger(mpc_.get(), test_dir_, "test_log");
    
    Eigen::Vector2d x0{1.0, 0.5};
    Eigen::VectorXd u0 = Eigen::VectorXd::Constant(1, 0.1);
    Eigen::VectorXd x_pred = Eigen::VectorXd::Zero(2 * 6);
    Eigen::VectorXd u_pred = Eigen::VectorXd::Zero(1 * 5);
    
    logger.logStep(0.0, x0, u0, x_pred, u_pred, 0.005, 0.001);
    logger.finalize();
    
    cnpy::npz_t my_npz = cnpy::npz_load((test_dir_ / "test_log.npz").string());
    
    // Check time
    auto it_time = my_npz.find("time");
    ASSERT_NE(it_time, my_npz.end());
    EXPECT_EQ(it_time->second.data<double>()[0], 0.0);
    
    // Check solve_times
    auto it_st = my_npz.find("solve_times");
    ASSERT_NE(it_st, my_npz.end());
    EXPECT_EQ(it_st->second.data<double>()[0], 0.005);
    EXPECT_EQ(it_st->second.data<double>()[1], 0.001);
    
    // Check x_curr
    auto it_xc = my_npz.find("x_curr");
    ASSERT_NE(it_xc, my_npz.end());
    EXPECT_EQ(it_xc->second.data<double>()[0], 1.0);
    EXPECT_EQ(it_xc->second.data<double>()[1], 0.5);
    
    // Check predicted shapes
    EXPECT_EQ(my_npz["x_pred"].shape[1], 6);
    EXPECT_EQ(my_npz["u_pred"].shape[1], 5);
    
    // Check metadata snapshot
    ASSERT_NE(my_npz.find("meta_state_dim"), my_npz.end());
    EXPECT_EQ(my_npz["meta_state_dim"].data<int>()[0], 2);
}
