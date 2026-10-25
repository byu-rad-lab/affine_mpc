#ifndef UTILS_HPP
#define UTILS_HPP

#include <Eigen/Core>
#include <gtest/gtest.h>
#include <iostream>
#include <stdexcept>

#define PRINT_MAT(A) (std::cout << #A << ":\n" << A << std::endl)

// Records a GoogleTest failure (and returns false) unless mat1 and mat2 have
// the same size and every element differs by at most delta. The return value
// also allows ASSERT_TRUE(expectEigenNear(...)) to stop the test.
template <typename Derived1, typename Derived2>
bool expectEigenNear(const Eigen::MatrixBase<Derived1>& mat1,
                     const Eigen::MatrixBase<Derived2>& mat2,
                     double delta)
{
  if (mat1.rows() != mat2.rows() || mat1.cols() != mat2.cols()) {
    ADD_FAILURE() << "size mismatch: " << mat1.rows() << "x" << mat1.cols()
                  << " vs " << mat2.rows() << "x" << mat2.cols();
    return false;
  }
  if (mat1.size() == 0)
    return true;
  const double max_diff{
      static_cast<double>((mat1 - mat2).cwiseAbs().maxCoeff())};
  if (!(max_diff <= delta)) { // also fails on NaN
    ADD_FAILURE() << "max |difference| " << max_diff << " exceeds " << delta;
    return false;
  }
  return true;
}

// Discrete model x_next = Ad x + Bd u + wd of an MPC object, read through its
// public propagateModel() (columns of Ad and Bd are exact up to the rounding of
// adding and subtracting wd).
struct DiscreteModel
{
  Eigen::MatrixXd Ad, Bd;
  Eigen::VectorXd wd;
};

template <typename MPC> DiscreteModel readModel(const MPC& mpc)
{
  const int n{mpc.getStateDim()}, m{mpc.getInputDim()};
  DiscreteModel model{Eigen::MatrixXd{n, n}, Eigen::MatrixXd{n, m},
                      Eigen::VectorXd{n}};
  const Eigen::VectorXd x_zero{Eigen::VectorXd::Zero(n)};
  const Eigen::VectorXd u_zero{Eigen::VectorXd::Zero(m)};
  mpc.propagateModel(x_zero, u_zero, model.wd);
  Eigen::VectorXd x_next{n};
  for (int i{0}; i < n; ++i) {
    mpc.propagateModel(Eigen::VectorXd::Unit(n, i), u_zero, x_next);
    model.Ad.col(i) = x_next - model.wd;
  }
  for (int j{0}; j < m; ++j) {
    mpc.propagateModel(x_zero, Eigen::VectorXd::Unit(m, j), x_next);
    model.Bd.col(j) = x_next - model.wd;
  }
  return model;
}

void expectInvalidArgumentWithMessage(const std::function<void()>& fn,
                                      const std::string& expected_substring)
{
  try {
    fn();
    FAIL() << "Expected std::invalid_argument";
  } catch (const std::invalid_argument& e) {
    EXPECT_TRUE(std::string(e.what()).find(expected_substring)
                != std::string::npos)
        << "Exception message was: " << e.what();
  } catch (...) {
    FAIL() << "Expected std::invalid_argument";
  }
}

void expectLogicErrorWithMessage(const std::function<void()>& fn,
                                 const std::string& expected_substring)
{
  try {
    fn();
    FAIL() << "Expected std::logic_error";
  } catch (const std::logic_error& e) {
    EXPECT_TRUE(std::string(e.what()).find(expected_substring)
                != std::string::npos)
        << "Exception message was: " << e.what();
  } catch (...) {
    FAIL() << "Expected std::logic_error";
  }
}

#endif // UTILS_HPP
