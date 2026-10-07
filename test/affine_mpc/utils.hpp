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
